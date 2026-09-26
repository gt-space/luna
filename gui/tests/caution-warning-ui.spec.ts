import { expect, Page, test } from "@playwright/test";

test.use({ headless: true });

// Exercise the real main window with an in-memory Tauri event bridge.
async function emit(page: Page, event: string, payload: unknown) {
  await page.evaluate(({ event, payload }) => (window as any).__testEmit(event, payload), { event, payload });
}
const state = (connected: boolean, source = "umbilical") => ({
  isConnected: connected, currentDataSource: source,
  alerts: [{ time: "12:00:00", agent: "GUI", message: "Example event" }],
});
const frame = { rolling: {}, gps: { has_fix: false }, reco: [{ ekf_blown_up: true }] };

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => {
    const host = window as any;
    const listeners: Record<string, number[]> = {};
    host.__TAURI_METADATA__ = { __windows: [{ label: "taskbar" }], __currentWindow: { label: "taskbar" } };
    host.__testEmit = (event: string, payload: unknown) => {
      for (const handler of listeners[event] ?? []) host[`_${handler}`]({ event, payload });
    };
    host.__TAURI_IPC__ = ({ callback, __tauriModule, message }: any) => {
      if (__tauriModule === "Event" && message.cmd === "listen") {
        (listeners[message.event] ??= []).push(message.handler);
      }
      host[`_${callback}`](null);
    };
  });
  await page.goto("http://localhost:1420");
  await expect(page.getByRole("heading", { name: "Caution & Warning" })).toBeVisible();
});

test("live faults, filtering, event log, stale data, and source changes", async ({ page }, testInfo) => {
  await expect(page.getByRole("heading", { name: "Server disconnected" })).toBeVisible();
  await emit(page, "state", state(true));
  await expect(page.getByRole("heading", { name: "Waiting for telemetry" })).toBeVisible();
  // Keep frames fresh while interacting with filters.
  await page.evaluate(frame => {
    const host = window as any;
    host.__streamTimer = setInterval(() => host.__testEmit("device_update", frame), 50);
  }, frame);
  await expect(page.getByRole("heading", { name: "RECO A estimator fault" })).toBeVisible();
  await expect(page.getByRole("status")).toHaveText("1 warning · 1 caution · 1 informational");
  await page.screenshot({ path: testInfo.outputPath("live-status.png") });
  await page.getByLabel("Show", { exact: true }).selectOption("caution");
  await expect(page.getByRole("heading", { name: "GPS has no fix" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "RECO A estimator fault" })).toHaveCount(0);
  await page.getByLabel("Show", { exact: true }).selectOption("all");
  await page.getByText("Event log (1)").click();
  await expect(page.getByText("Example event")).toBeVisible();
  await page.evaluate(() => clearInterval((window as any).__streamTimer));
  await expect(page.getByRole("heading", { name: "Telemetry is stale" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "RECO A estimator fault" })).toHaveCount(0);
  await emit(page, "state", state(true, "tel"));
  await expect(page.getByRole("heading", { name: "Waiting for telemetry" })).toBeVisible();
  await emit(page, "state", state(false, "tel"));
  await expect(page.getByRole("heading", { name: "Server disconnected" })).toBeVisible();
});

test("panel remains inside the main window at compact and narrow sizes", async ({ page }) => {
  for (const size of [{ width: 640, height: 350 }, { width: 400, height: 600 }]) {
    await page.setViewportSize(size);
    const bounds = await page.getByRole("region", { name: "Caution & Warning" }).boundingBox();
    expect(bounds).not.toBeNull();
    expect(bounds!.x).toBeGreaterThanOrEqual(0);
    expect(bounds!.x + bounds!.width).toBeLessThanOrEqual(size.width);
    await expect(page.getByRole("heading", { name: "Server disconnected" })).toBeVisible();
  }
});
