/**
 * Control-loop timing (examples/mock-apps/timing): each arm at its shipped rate on simulated motors behind the fake
 * adapter, in the page and in a Web Worker. The achieved rate must be within 5% of the target (8% on Windows and on CI runners,
 * unchecked on macOS); the tail bounds are generous so a loaded machine does not flake them. The numbers are attached
 * to the report and printed.
 */

import { expect, openApp, test } from "./fixtures.ts";

interface Timing {
  product: string;
  where: string;
  arm: string;
  targetHz: number;
  achievedHz: number;
  medianMs: number;
  p99Ms: number;
  maxMs: number;
  ticks: number;
  seconds: number;
}

test("each arm's control loop keeps its shipped rate, alone and two arms at once, in the page and in a worker", async ({ page }, testInfo) => {
  test.setTimeout(180_000);
  await openApp(page, "/timing/");
  await page.fill("#seconds", "4");
  await page.click("#measure");
  await expect(page.locator("#message")).toHaveText("Done.", { timeout: 150_000 });
  const results = await page.$$eval("#results tr", (rows) => rows.map((r) => JSON.parse((r as HTMLElement).dataset.result!)) as Timing[]);
  expect(results.map((r) => `${r.product}-${r.where}-${r.arm}`)).toEqual(
    ["metal", "maker"].flatMap((p) => [`${p}-page-single`, `${p}-worker-single`, `${p}-page-left`, `${p}-page-right`, `${p}-worker-left`, `${p}-worker-right`]),
  );
  const lines = results.map((r) =>
    `${r.product.padEnd(5)} ${r.where.padEnd(6)} ${(r.arm === "single" ? "one arm" : `2 arms, ${r.arm}`).padEnd(13)} target ${r.targetHz} Hz, got ${r.achievedHz.toFixed(1)} Hz; ` +
    `period median ${r.medianMs.toFixed(2)} ms, p99 ${r.p99Ms.toFixed(2)} ms, max ${r.maxMs.toFixed(2)} ms (${r.ticks} ticks)`);
  console.log(lines.join("\n"));
  await testInfo.attach("loop-timing.txt", { body: lines.join("\n"), contentType: "text/plain" });
  test.skip(
    process.platform === "darwin",
    "the shared macOS runners stall the page for 30-90 ms at a time (measured 38-73 Hz of 100 and 91-182 Hz of 200); the bounds hold on Linux and Windows",
  );
  // The shared Windows runners' workers lag a little (measured down to 190.5 Hz of 200) and an occasional run's
  // p99 reaches 40 ms: 8% and period + 35 ms there; Linux keeps 5% and period + 15 ms.
  const windows = process.platform === "win32";
  // Shared CI runners: two 200 Hz loops in one worker have measured up to 5.8 % slow on a Linux runner.
  const rateTolerance = windows || process.env.CI ? 0.08 : 0.05;
  for (const r of results) {
    const period = 1000 / r.targetHz;
    expect(Math.abs(r.achievedHz - r.targetHz) / r.targetHz, `${r.product} ${r.where} ${r.arm} rate`).toBeLessThan(rateTolerance);
    expect(r.ticks, `${r.product} ${r.where} ${r.arm} ticks`).toBeGreaterThan(0.9 * r.targetHz * r.seconds);
    expect(r.medianMs, `${r.product} ${r.where} ${r.arm} median period`).toBeLessThan(1.25 * period);
    expect(r.p99Ms, `${r.product} ${r.where} ${r.arm} p99 period`).toBeLessThan(period + (windows ? 35 : 15));
    expect(r.maxMs, `${r.product} ${r.where} ${r.arm} longest period`).toBeLessThan(period + 100);
  }
});
