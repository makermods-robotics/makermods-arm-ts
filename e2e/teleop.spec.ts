/**
 * Teleoperation (examples/mock-apps/teleop): a simulated Star Arm 102 leader, its servos turned by the page's
 * sliders and read through the real FashionStar protocol code, drives a follower on the fake gs_usb adapter through
 * Teleop, for Metal and for Maker.
 */

import type { Page } from "@playwright/test";
import { expect, hook, maxAbsDiff, openApp, peakSpeed, sample, slide, test } from "./fixtures.ts";

for (const product of ["metal", "maker"] as const) {
  test.describe(`${product} teleop`, () => {
    async function ready(page: Page): Promise<void> {
      await openApp(page, `/teleop/?mock=${product}`);
      await page.selectOption("#product", product);
      await page.click("#connect");
      await expect(page.locator("#message")).toContainText("Connected to FAKE-");
      await page.click("#enable");
      await expect(page.locator("#state")).toHaveText("enabled");
    }
    const positions = (page: Page) => hook<number[]>(page, "positions");
    const targets = async (page: Page) => JSON.parse((await page.locator("#targets").textContent())!) as number[];
    /** Follower arm joints (not the gripper) on the leader's targets. */
    const following = async (page: Page) => maxAbsDiff((await positions(page)).slice(0, 6), (await targets(page)).slice(0, 6));

    test("syncs slowly from a large gap, then follows the sliders", async ({ page }) => {
      await ready(page);
      await slide(page, "#star-servo-0", 90); // shoulder pan far from the follower before teleop starts
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("syncing");
      const syncing = await sample(page, "positions", 1500);
      expect(peakSpeed(syncing, 0), "sync speed, rad/s (Teleop's syncVelocity is 0.3)").toBeLessThan(0.4);
      expect(peakSpeed(syncing, 0)).toBeGreaterThan(0.15);
      await expect(page.locator("#teleop")).toHaveText("following", { timeout: 15_000 });
      await expect(page.locator("#gap")).toHaveText(""); // reported once stopped
      await expect.poll(() => following(page)).toBeLessThan(0.02);
      for (const [servo, deg] of [[1, -30], [3, 20], [5, 45]] as const) {
        await slide(page, `#star-servo-${servo}`, deg);
        await expect.poll(() => following(page)).toBeLessThan(0.02);
      }
      await page.click("#stop");
      await expect(page.locator("#teleop")).toHaveText("stopped: stopped");
      const gap = Number((await page.locator("#gap").textContent())!.match(/startup gap ([\d.]+) rad/)![1]);
      expect(gap).toBeGreaterThan(0.8); // Teleop's syncThreshold
    });

    test("the speed cap holds", async ({ page }) => {
      await ready(page);
      await page.fill("#cap", "0.5");
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following");
      await slide(page, "#star-servo-0", 60); // a 1 rad jump of the leader
      const moving = await sample(page, "positions", 1500);
      const peak = peakSpeed(moving, 0);
      expect(peak, "follower speed under a 0.5 rad/s cap").toBeLessThan(0.6);
      expect(peak).toBeGreaterThan(0.3);
      await expect.poll(() => following(page), { timeout: 10_000 }).toBeLessThan(0.02);
    });

    test("Stop leaves the follower holding and a second Start resumes", async ({ page }) => {
      await ready(page);
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following");
      await page.click("#stop");
      await expect(page.locator("#message")).toHaveText("Teleop stopped; the follower is holding.");
      const held = await positions(page);
      await slide(page, "#star-servo-0", 40); // not followed while stopped
      await page.waitForTimeout(500);
      expect(maxAbsDiff(await positions(page), held)).toBeLessThan(1e-3);
      expect(await hook(page, "enabled")).toEqual(new Array(7).fill(true));
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following", { timeout: 15_000 });
      await expect.poll(() => following(page), { timeout: 10_000 }).toBeLessThan(0.02);
      expect(Math.abs((await positions(page))[0]! - held[0]!)).toBeGreaterThan(0.3);
    });

    test("a leader that stops answering ends teleop, the follower holding", async ({ page }) => {
      await ready(page);
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following");
      await page.locator("#star-online-2").uncheck(); // the elbow servo drops off the bus
      await expect(page.locator("#teleop")).toContainText("stopped: leader_lost", { timeout: 10_000 });
      // The app names the servo from the leader's last bus error.
      await expect(page.locator("#message")).toContainText("servo 2 lost");
      const held = await positions(page);
      await page.waitForTimeout(500);
      expect(maxAbsDiff(await positions(page), held)).toBeLessThan(1e-3);
      await expect(page.locator("#state")).toHaveText("enabled");
      // Back on the bus, a new Start reconnects the leader.
      await page.locator("#star-online-2").check();
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following", { timeout: 15_000 });
    });

    test("a follower fault ends teleop", async ({ page }) => {
      await ready(page);
      await page.click("#start");
      await expect(page.locator("#teleop")).toHaveText("following");
      await hook(page, "fault", "wrist_flex");
      await expect(page.locator("#teleop")).toContainText("stopped: follower_fault", { timeout: 10_000 });
      await expect(page.locator("#teleop")).toContainText("wrist_flex");
      await expect(page.locator("#state")).toHaveText("fault");
      await expect(page.locator("#start")).toBeDisabled();
    });
  });
}
