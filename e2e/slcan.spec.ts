/**
 * The arm console on an SLCAN adapter: SlcanSerialBackend over a fake CANable on a fake serial port (navigator.serial
 * mocked), simulated Maker motors behind it. The fake adapter records any two frames closer than canable-fw allows.
 */

import { expect, hook, maxAbsDiff, openApp, test } from "./fixtures.ts";

test("maker arm over SLCAN: connect, enable, move, rest", async ({ page }) => {
  test.setTimeout(60_000);
  await openApp(page, "/arm-console/?mock=maker&link=slcan");
  await page.selectOption("#product", "maker");
  await expect(page.locator("#link")).toHaveValue("slcan");
  await page.click("#connect");
  // The adapter's startup delay (2.5 s: USB serial bridges may reset on open) comes first.
  await expect(page.locator("#message")).toHaveText("Connected to SLCAN adapter usb:AD50:60C4: 7 joints.", { timeout: 15_000 });
  const rest = await hook<number[]>(page, "positions");
  await page.click("#enable");
  await expect(page.locator("#state")).toHaveText("enabled");
  expect(await hook(page, "enabled")).toEqual(new Array(7).fill(true));
  await page.selectOption("#joint", "wrist_flex");
  await page.fill("#delta", "0.3");
  await page.click("#move");
  await expect(page.locator("#message")).toContainText("Arrived: wrist_flex", { timeout: 15_000 });
  const target = JSON.parse((await page.locator("#target").textContent())!) as number[];
  await expect.poll(async () => maxAbsDiff(await hook<number[]>(page, "positions"), target)).toBeLessThan(0.005);
  await page.click("#rest");
  await expect(page.locator("#message")).toHaveText("At rest, torque released.", { timeout: 20_000 });
  expect(maxAbsDiff(await hook<number[]>(page, "positions"), rest)).toBeLessThan(0.02);
  expect(await hook(page, "enabled")).toEqual(new Array(7).fill(false));
  await page.click("#disconnect");
  await expect(page.locator("#message")).toHaveText("Disconnected.");
  const commands = await hook<string[]>(page, "commands");
  expect(commands.slice(0, 4)).toEqual(["C", "S8", "A0", "O"]);
  expect(commands.at(-1)).toBe("C");
  expect(await hook(page, "isOpen")).toBe(false);
  expect(await hook<number>(page, "frames")).toBeGreaterThan(100);
  expect(await hook(page, "protocolErrors")).toEqual([]);
});
