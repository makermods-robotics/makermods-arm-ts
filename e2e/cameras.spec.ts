/**
 * Cameras (examples/mock-apps/cameras): the SDK's Camera and listCameras against Chromium's synthetic camera
 * (--use-fake-device-for-media-stream; no real camera is opened).
 */

import { expect, jpegSize, openApp, test } from "./fixtures.ts";

test("list, open, read frames with their age, and encode a JPEG of the requested size", async ({ page }) => {
  await openApp(page, "/cameras/");
  await page.click("#list");
  await expect(page.locator("#message")).toContainText("camera(s).");
  const count = await page.locator("#cameras tr").count();
  expect(count).toBeGreaterThanOrEqual(1);
  await page.click("#open");
  await expect(page.locator("#message")).toContainText("Opened");
  // Once the page has camera permission the list carries device ids and labels.
  await page.click("#list");
  await expect(page.locator("#cameras tr").first().locator("code")).toHaveText(/^device:/);

  for (let i = 1; i <= 3; i++) {
    await page.click("#read");
    await expect(page.locator("#reads")).toHaveText(String(i));
  }
  expect(Number(await page.locator("#frame").getAttribute("data-width"))).toBe(640);
  expect(Number(await page.locator("#frame").getAttribute("data-height"))).toBe(480);
  await expect.poll(async () => Number(await page.locator("#age").getAttribute("data-ms"))).toBeLessThan(200);

  await page.fill("#jpeg-width", "320");
  await page.fill("#jpeg-height", "176");
  await page.click("#jpeg");
  await expect(page.locator("#jpeg-info")).toContainText("decodes as 320x176");
  const bytes = Buffer.from((await page.locator("#jpeg-info").getAttribute("data-base64"))!, "base64");
  expect([...bytes.subarray(0, 3)]).toEqual([0xff, 0xd8, 0xff]);
  expect([...bytes.subarray(-2)]).toEqual([0xff, 0xd9]);
  expect(jpegSize(bytes)).toEqual({ width: 320, height: 176 });

  await page.click("#close");
  await expect(page.locator("#message")).toHaveText("Closed.");
  await expect(page.locator("#read")).toBeDisabled();
});
