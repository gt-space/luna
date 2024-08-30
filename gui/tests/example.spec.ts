import { test, expect } from '@playwright/test';

test('should contain the word "Welcome" on the homepage', async ({ page }) => {
  // Navigate to the homepage of your application
  await page.goto('http://localhost:1420'); // Adjust the URL to match your app

  // Get the page content or the specific element containing the text
  const pageContent = await page.content(); // Gets the entire HTML content of the page

  // Assert that the word "Welcome" is present in the page content
  expect(pageContent).toContain('System');
});