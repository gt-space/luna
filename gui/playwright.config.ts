import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './tests',
  timeout: 30000,
  use: {
    headless: false, // Set to true for CI environments
    ignoreHTTPSErrors: true,
    viewport: { width: 1280, height: 720 },
  },
});