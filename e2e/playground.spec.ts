/**
 * The mock playground (examples/mock-apps/playground): a camera (Chromium's synthetic one, through the SDK's Camera), the Dream client in the
 * page pointed at a proxy the test runs (the Dream client's createDreamProxy + nodeHandler, holding the key) in front
 * of a stand-in gateway answering recorded chunks, the chunks solved by the Metal arm's trajectory IK and played to
 * a simulated Metal arm on the fake adapter. Skipped when ../dreamlabs-ts is not checked out.
 */

import type { Page } from "@playwright/test";
import { type DreamStack, hasDreamlabs, startDreamStack } from "./dream-server.ts";
import { expect, hook, jpegSize, maxAbsDiff, openApp, test } from "./fixtures.ts";

test.skip(!hasDreamlabs, "needs the Dream client checked out at ../dreamlabs-ts");

let stack: DreamStack;

test.beforeEach(async ({ baseURL }) => {
  stack = await startDreamStack(new URL(baseURL!).origin);
});

test.afterEach(async () => {
  await stack.close();
  expect(stack.escaped, "upstream calls the proxy tried to make past the stand-in").toEqual([]);
});

/** Every request and answer the page saw, for the key check. */
function watchTraffic(page: Page): { seen: string[] } {
  const seen: string[] = [];
  page.on("request", (r) => seen.push(`${r.method()} ${r.url()}\n${JSON.stringify(r.headers())}\n${r.postData() ?? ""}`));
  page.on("response", async (r) => {
    seen.push(JSON.stringify(await r.allHeaders()));
    if (r.url().startsWith(stack.proxyUrl)) seen.push(await r.text().catch(() => ""));
  });
  return { seen };
}

async function ready(page: Page): Promise<void> {
  await openApp(page, "/playground/?mock=metal");
  await page.click("#connect");
  await expect(page.locator("#message")).toHaveText("Connected to FAKE-METAL-1.");
  await page.fill("#proxy", stack.proxyUrl);
}

const positions = (page: Page) => hook<number[]>(page, "positions");
const enabled = (page: Page) => hook<boolean[]>(page, "enabled");
const getActions = () => stack.received.filter((r) => r.path === "/v1/get_action");

test("one full episode runs and the arm follows the chunks", async ({ page }) => {
  await ready(page);
  const start = await positions(page);
  await page.click("#run");
  await expect(page.locator("#message")).toHaveText(/^Episode done: 3 chunks, \d+ rows; holding\.$/, { timeout: 45_000 });

  // The policy saw camera frames (JPEG) and the arm's tool pose, through the proxy.
  const sent = getActions();
  expect(sent).toHaveLength(3);
  for (const r of sent) {
    const observation = (JSON.parse(r.body) as { observation: Record<string, unknown> }).observation;
    for (const view of ["video.overhead", "video.wrist_l", "video.wrist_r"]) {
      const frames = observation[view] as string[];
      expect(Array.isArray(frames) && frames.length > 0, view).toBe(true);
      expect(frames.every((f) => f.startsWith("/9j/")), `${view} is base64 JPEG`).toBe(true);
      // The camera's frames, through the SDK's Camera and Frame.toJpeg(), at the size the model is sent.
      expect(jpegSize(Buffer.from(frames.at(-1)!, "base64")), view).toEqual({ width: 320, height: 176 });
    }
    expect(observation.prompt).toBe("pick up the cube");
    expect((observation["state.right_eef_xyz"] as number[]).every(Number.isFinite)).toBe(true);
    expect(observation.embodiment).toBe("other_embodiment");
  }

  // Every row of every chunk was played, and the arm went where the last one said.
  const rows = Number(await page.locator("#rows").textContent());
  expect(rows).toBeGreaterThanOrEqual(48);
  const last = JSON.parse((await page.locator("#last").textContent())!) as number[];
  await expect.poll(async () => maxAbsDiff((await positions(page)).slice(0, 6), last.slice(0, 6))).toBeLessThan(0.005);
  expect(maxAbsDiff(await positions(page), start)).toBeGreaterThan(0.5);
  // The first chunk was recorded from the ready pose the arm starts at: the IK reaches it closely. (Later recorded
  // chunks do not continue it, so the speed cap can leave them short; their misses are only reported.)
  const misses = JSON.parse((await page.locator("#ik").getAttribute("data-misses"))!) as number[];
  expect(misses).toHaveLength(3);
  expect(misses[0]!, "first chunk's IK position miss, m").toBeLessThan(0.03);
  const tracking = Number((await page.locator("#tracking").textContent())!.replace(" rad", ""));
  expect(tracking, "measured vs commanded").toBeLessThan(0.1);
  await expect(page.locator("#state")).toHaveText("enabled");
  expect(await enabled(page)).toEqual(new Array(7).fill(true));
});

test("the proxy holds the key and the page never sees it", async ({ page }) => {
  const traffic = watchTraffic(page);
  await ready(page);
  await page.fill("#chunks", "2");
  await page.click("#run");
  await expect(page.locator("#message")).toContainText("Episode done: 2 chunks", { timeout: 45_000 });
  // Upstream, every call carried the key...
  expect(stack.received.length).toBeGreaterThan(0);
  for (const r of stack.received) expect(r.headers.authorization).toBe(`Bearer ${stack.key}`);
  // ...which appears nowhere the page could read: its requests, the answers, the DOM, its storage.
  const pageRequests = traffic.seen.filter((t) => t.includes(stack.proxyUrl));
  expect(pageRequests.length).toBeGreaterThan(0);
  for (const t of traffic.seen) expect(t.includes(stack.key)).toBe(false);
  expect(traffic.seen.some((t) => /authorization/i.test(t.split("\n")[1] ?? ""))).toBe(false);
  const visible = await page.evaluate(() => document.documentElement.outerHTML + JSON.stringify({ ...localStorage }) + JSON.stringify({ ...sessionStorage }));
  expect(visible.includes(stack.key)).toBe(false);
  expect(visible.includes("dk_mock_")).toBe(false);
});

test("a gateway error surfaces in the UI and the arm holds", async ({ page }) => {
  await ready(page);
  stack.mode = "error";
  await page.click("#run");
  await expect(page.locator("#message")).toContainText("Policy error: DreamServerError", { timeout: 45_000 });
  await expect(page.locator("#message")).toContainText("DA-5000");
  await expect(page.locator("#message")).toContainText("the arm is holding");
  await expect(page.locator("#phase")).toHaveText("stopped on an error");
  const held = await positions(page);
  await page.waitForTimeout(1000);
  expect(maxAbsDiff(await positions(page), held)).toBeLessThan(1e-3);
  expect(await enabled(page)).toEqual(new Array(7).fill(true));
  await expect(page.locator("#state")).toHaveText("enabled");
  await expect(page.locator("#run")).toBeEnabled();
});

test("Stop mid-episode returns the arm to rest and releases it", async ({ page }) => {
  await ready(page);
  const rest = await positions(page);
  await page.fill("#chunks", "50");
  await page.click("#run");
  await expect.poll(() => getActions().length, { timeout: 30_000 }).toBeGreaterThanOrEqual(2);
  expect(maxAbsDiff(await positions(page), rest)).toBeGreaterThan(0.3);
  await page.click("#stop");
  await expect(page.locator("#message")).toHaveText("Stopped: at rest, torque released.", { timeout: 30_000 });
  // The gripper is not part of the return (return_to_rest: false in the Metal profile): the six arm joints are.
  expect(maxAbsDiff((await positions(page)).slice(0, 6), rest.slice(0, 6))).toBeLessThan(0.03);
  expect(await enabled(page)).toEqual(new Array(7).fill(false));
  // The session was released, and nothing more was asked for.
  expect(stack.received.some((r) => r.path === "/v1/end_session")).toBe(true);
  const asked = getActions().length;
  await page.waitForTimeout(1000);
  expect(getActions().length).toBe(asked);
});
