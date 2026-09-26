import { expect, test } from "@playwright/test";
import { readFileSync, readdirSync } from "node:fs";
import { resolve } from "node:path";
import { activeMessages, loadRules, MessageRule, StatusContext } from "../src/caution-warning/rules";

const configDir = resolve(__dirname, "../src/caution-warning/config");
const configured = loadRules(Object.fromEntries(readdirSync(configDir).map(name => [name, readFileSync(resolve(configDir, name), "utf8")])));
const live: StatusContext = {
  connected: true, hasTelemetry: true, telemetryFresh: true,
  telemetryAgeMs: 0, dataSource: "umbilical", telemetry: {},
};
const ids = (context: StatusContext) => activeMessages(configured.rules, context).map(rule => rule.id);
const rule = (overrides: Partial<MessageRule> = {}): MessageRule => ({
  id: "example", severity: "caution", title: "Example", message: "Example message",
  subsystem: "Test", source: "telemetry", all: [{ path: "telemetry.value", operator: "gt", value: 10 }], ...overrides,
});

test("shipped configuration is valid and distinguishes disconnected, waiting, stale and live", () => {
  expect(configured.errors).toEqual([]);
  expect(ids({ ...live, connected: false, hasTelemetry: false, telemetryFresh: false })).toEqual(["server-disconnected"]);
  expect(ids({ ...live, hasTelemetry: false, telemetryFresh: false })).toEqual(["telemetry-waiting"]);
  expect(ids({ ...live, telemetryFresh: false })).toEqual(["telemetry-stale"]);
  expect(ids(live)).toEqual(["telemetry-live"]);
});

test("faults sort by severity, clear on recovery, and disappear when telemetry is stale", () => {
  const context = { ...live, telemetry: { gps: { has_fix: false }, reco: [{ ekf_blown_up: true }] } };
  expect(ids(context)).toEqual(["reco-a-ekf-fault", "gps-no-fix", "telemetry-live"]);
  expect(ids({ ...context, telemetryFresh: false })).toEqual(["telemetry-stale"]);
  expect(ids({ ...live, telemetry: { gps: { has_fix: true }, reco: [{ ekf_blown_up: false }] } })).toEqual(["telemetry-live"]);
});

test("missing values never trigger comparisons, including inequality", () => {
  for (const value of [undefined, null, NaN, Infinity]) {
    const context = { ...live, telemetry: { value } };
    expect(activeMessages([rule()], context)).toEqual([]);
    expect(activeMessages([rule({ all: [{ path: "telemetry.value", operator: "ne", value: 10 }] })], context)).toEqual([]);
    expect(activeMessages([rule({ all: [{ path: "telemetry.value", operator: "missing" }] })], context)).toHaveLength(1);
  }
});

test("numeric comparisons use strict types and correct boundaries", () => {
  const context = { ...live, telemetry: { value: 10 } };
  for (const [operator, expected] of [["gt", 0], ["gte", 1], ["lt", 0], ["lte", 1], ["eq", 1], ["ne", 0]] as const) {
    expect(activeMessages([rule({ all: [{ path: "telemetry.value", operator, value: 10 }] })], context)).toHaveLength(expected);
  }
  expect(activeMessages([rule()], { ...live, telemetry: { value: "11" } })).toEqual([]);
});

test("all conditions must match and disabled rules stay inactive", () => {
  const combined = rule({ all: [
    { path: "telemetry.value", operator: "exists" },
    { path: "dataSource", operator: "eq", value: "tel" },
  ] });
  expect(activeMessages([combined], { ...live, telemetry: { value: 0 } })).toEqual([]);
  expect(activeMessages([combined], { ...live, dataSource: "tel", telemetry: { value: 0 } })).toHaveLength(1);
  expect(activeMessages([rule({ enabled: false })], { ...live, telemetry: { value: 20 } })).toEqual([]);
});

test("bad JSON, duplicate ids, and invalid rules are reported without dropping valid rules", () => {
  const result = loadRules({
    "a.json": JSON.stringify({ version: 1, messages: [rule(), rule(), { ...rule(), id: "bad", severity: "urgent" }] }),
    "b.json": "not JSON",
    "c.json": JSON.stringify({ version: 2, messages: [] }),
  });
  expect(result.rules).toHaveLength(1);
  expect(result.errors).toHaveLength(4);
});

test("validation rejects unsafe paths and system rules that bypass telemetry freshness", () => {
  for (const candidate of [
    rule({ all: [{ path: "telemetry.__proto__.value", operator: "exists" }] }),
    rule({ source: "system" }),
    rule({ all: [{ path: "telemetry.value", operator: "gt", value: "10" }] }),
    rule({ all: [] }),
  ]) {
    const result = loadRules({ "test.json": JSON.stringify({ version: 1, messages: [candidate] }) });
    expect(result.rules).toEqual([]);
    expect(result.errors.length).toBeGreaterThan(0);
  }
});
