/**
 * Arm-to-arm following (examples/mock-apps/arm-follow): a floated leader arm (ArmLeader) on one fake adapter and a
 * follower on another; the test moves the leader's simulated motors as a hand would.
 */

import { expect, hook, maxAbsDiff, openApp, test } from "./fixtures.ts";

for (const product of ["metal", "maker"] as const) {
  test(`${product}: the follower copies a leader moved by hand`, async ({ page }) => {
    await openApp(page, `/arm-follow/?mock=${product}&adapters=2`);
    await page.selectOption("#product", product);
    await page.click("#connect-leader");
    await expect(page.locator("#leader-port")).toHaveText(`FAKE-${product.toUpperCase()}-1`);
    await page.click("#connect-follower");
    await expect(page.locator("#follower-port")).toHaveText(`FAKE-${product.toUpperCase()}-2`);
    const leaderSerial = `FAKE-${product.toUpperCase()}-1`;
    const followerSerial = `FAKE-${product.toUpperCase()}-2`;
    const of = (serial: string) => hook<number[]>(page, "callOn", serial, "positions");

    await page.click("#start");
    await expect(page.locator("#teleop")).toHaveText("following", { timeout: 15_000 });
    expect(await hook(page, "callOn", leaderSerial, "enabled")).toEqual(new Array(7).fill(true)); // floated, not limp
    // The hand moves the leader through a few poses; the follower's arm joints come along.
    const start = await of(leaderSerial);
    const poses = [
      start.map((q, i) => (i === 0 ? q + 0.4 : q)),
      start.map((q, i) => (i === 0 ? q + 0.4 : i === 3 ? q + 0.3 : q)),
      start.map((q, i) => (i === 5 ? q - 0.5 : q)),
    ];
    for (const pose of poses) {
      await hook(page, "callOn", leaderSerial, "setPositions", pose);
      await page.waitForTimeout(150);
      // The leader stays where the hand left it (floating: no stiffness pulls it back).
      expect(maxAbsDiff((await of(leaderSerial)).slice(0, 6), pose.slice(0, 6))).toBeLessThan(1e-3);
      await expect.poll(async () => maxAbsDiff((await of(followerSerial)).slice(0, 6), pose.slice(0, 6)), { timeout: 10_000 }).toBeLessThan(0.02);
    }

    await page.click("#stop");
    await expect(page.locator("#teleop")).toHaveText("stopped: stopped");
    const held = await of(followerSerial);
    await hook(page, "callOn", leaderSerial, "setPositions", start);
    await page.waitForTimeout(400);
    expect(maxAbsDiff(await of(followerSerial), held)).toBeLessThan(1e-3);

    await page.click("#estop");
    await expect(page.locator("#message")).toHaveText("E-stop: torque released on both arms.");
    for (const serial of [leaderSerial, followerSerial]) {
      expect(await hook(page, "callOn", serial, "enabled")).toEqual(new Array(7).fill(false));
    }
  });
}

