/**
 * The SO-101 console (examples/mock-apps/so101) on a simulated SO-101 behind a fake CH343 serial port: the port
 * chooser, calibration by hand (the test moves the limp joints, as a person would), enable, move, wiggle, rest, and
 * a servo fault. The servos start at factory settings, and the browser has no calibration stored.
 */

import type { Page } from "@playwright/test";
import { expect, hook, openApp, test } from "./fixtures.ts";

const torque = (page: Page) => hook<number[]>(page, "so101.torque");
const positions = (page: Page) => hook<number[]>(page, "so101.positions");

async function connect(page: Page): Promise<void> {
  await openApp(page, "/so101/?mock=so101");
  await page.click("#connect");
  await expect(page.locator("#message")).toContainText("Connected: 6 servos.");
  await expect(page.locator("#state")).toHaveText("connected");
}

async function calibrate(page: Page): Promise<void> {
  await page.click("#calibrate");
  await expect(page.locator("#calibration")).toContainText("recording");
  await expect(page.locator("#save")).toBeEnabled();
  await hook(page, "so101.sweep");
  await expect(page.locator("#ranges tr")).toHaveCount(6);
  await page.click("#save");
  await expect(page.locator("#message")).toHaveText("Calibration saved as 'so101_left'.", { timeout: 20_000 });
  await expect(page.locator("#calibration")).toHaveText("'so101_left' loaded");
}

async function enable(page: Page): Promise<void> {
  await page.click("#enable");
  await expect(page.locator("#message")).toHaveText("Holding the present pose.");
  await expect(page.locator("#state")).toHaveText("enabled");
}

test("enable is refused until the arm is calibrated, with nothing switched on", async ({ page }) => {
  await connect(page);
  await expect(page.locator("#message")).toContainText("Calibrate before enabling.");
  await expect(page.locator("#calibration")).toContainText("calibrate it first");
  await page.click("#enable");
  await expect(page.locator("#message")).toContainText("calibrate it first");
  await expect(page.locator("#state")).toHaveText("connected");
  expect(await torque(page)).toEqual([0, 0, 0, 0, 0, 0]);
});

test("calibrate by hand, enable, move and return to rest", async ({ page }) => {
  test.setTimeout(90_000);
  await connect(page);
  await test.step("calibrate", async () => {
    await calibrate(page);
    const saved = await page.evaluate(() => Object.keys(localStorage).filter((k) => k.includes("so101_left")));
    expect(saved.length).toBeGreaterThan(0);
    expect(await torque(page)).toEqual([0, 0, 0, 0, 0, 0]);
  });
  const rest = await positions(page);
  await test.step("enable", async () => {
    await enable(page);
    expect(await torque(page)).toEqual([1, 1, 1, 1, 1, 1]);
  });
  await test.step("move", async () => {
    await page.selectOption("#joint", "shoulder_pan");
    await page.fill("#delta", "0.3");
    await page.click("#move");
    await expect(page.locator("#message")).toContainText("Arrived: shoulder_pan", { timeout: 20_000 });
    const target = JSON.parse((await page.locator("#target").textContent())!) as number[];
    await expect.poll(async () => Math.abs((await positions(page))[0]! - target[0]!)).toBeLessThan(0.03);
    expect((await positions(page))[0]! - rest[0]!).toBeGreaterThan(0.25);
  });
  await test.step("rest", async () => {
    await page.click("#rest");
    await expect(page.locator("#message")).toHaveText("At rest, torque released.", { timeout: 30_000 });
    const back = await positions(page);
    for (let i = 0; i < 5; i++) expect(Math.abs(back[i]! - rest[i]!), `joint ${i}`).toBeLessThan(0.06);
    expect(await torque(page)).toEqual([0, 0, 0, 0, 0, 0]);
    await expect(page.locator("#state")).toHaveText("connected");
  });
});

test("wiggle moves the gripper and lets it go again", async ({ page }) => {
  await connect(page);
  const before = (await hook<number[]>(page, "so101.goals", 6)).length;
  await page.click("#wiggle");
  await expect(page.locator("#message")).toContainText("Wiggled the gripper", { timeout: 15_000 });
  const goals = (await hook<number[]>(page, "so101.goals", 6)).slice(before);
  // Held where it was, three times up and down, then back.
  expect(goals.length).toBeGreaterThanOrEqual(8);
  expect(new Set(goals).size).toBeGreaterThanOrEqual(3);
  expect(await torque(page)).toEqual([0, 0, 0, 0, 0, 0]);
  await expect(page.locator("#state")).toHaveText("connected");
});

test("a servo fault puts the arm in FAULT holding; E-stop lets go", async ({ page }) => {
  test.setTimeout(90_000);
  await connect(page);
  await calibrate(page);
  await enable(page);
  await hook(page, "so101.servoError", 2, 0x04);
  await expect(page.locator("#state")).toHaveText("fault");
  await expect(page.locator("#fault")).toContainText("shoulder_lift");
  await expect(page.locator("#fault")).toContainText("overheat");
  await expect(page.locator('#joints tr[data-joint="shoulder_lift"] .fault')).toHaveText("overheat");
  expect(await torque(page)).toEqual([1, 1, 1, 1, 1, 1]); // holding, not dropped
  await page.click("#rest");
  await expect(page.locator("#message")).toContainText("the arm keeps holding");
  await page.click("#estop");
  await expect(page.locator("#message")).toHaveText("E-stop: torque released on every servo.");
  expect(await torque(page)).toEqual([0, 0, 0, 0, 0, 0]);
});
