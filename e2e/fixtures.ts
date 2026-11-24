/**
 * What every browser test runs under: nothing may leave the machine (every request to another host is recorded and
 * aborted, and the test fails if there was one), the real WebUSB and Web Serial are replaced by tripwires before any
 * page script runs (the mock hardware replaces them in turn; a call that reaches a tripwire fails the test), and an
 * uncaught page error fails the test.
 */

import { test as base, expect, type Page } from "@playwright/test";

const LOCAL = new Set(["127.0.0.1", "localhost", "[::1]"]);

export interface Guard {
  /** Requests to anything but this machine, which were aborted. */
  blocked: string[];
  /** Uncaught errors in the page. */
  pageErrors: string[];
  /** console.error lines (the SDK reports faults there; tests read them, they do not fail on them). */
  consoleErrors: string[];
  /** Page errors a test expects, matched and removed before the final check. */
  allowPageErrors: RegExp[];
}

export const test = base.extend<{ guard: Guard }>({
  guard: [
    async ({ context, page }, use) => {
      const guard: Guard = { blocked: [], pageErrors: [], consoleErrors: [], allowPageErrors: [] };
      await context.route("**/*", (route) => {
        const url = new URL(route.request().url());
        if (url.protocol === "data:" || url.protocol === "blob:" || LOCAL.has(url.hostname)) return route.continue();
        guard.blocked.push(route.request().url());
        return route.abort("blockedbyclient");
      });
      await context.addInitScript(() => {
        const tripped: string[] = [];
        (window as unknown as { __tripped: string[] }).__tripped = tripped;
        const tripwire = (api: string) => {
          const fail = (method: string) => () => {
            tripped.push(`${api}.${method}`);
            return Promise.reject(new Error(`the real navigator.${api} is off limits in tests (${method})`));
          };
          const target = new EventTarget();
          return Object.assign(target, {
            requestDevice: fail("requestDevice"),
            getDevices: fail("getDevices"),
            requestPort: fail("requestPort"),
            getPorts: fail("getPorts"),
          });
        };
        Object.defineProperty(navigator, "usb", { value: tripwire("usb"), configurable: true });
        Object.defineProperty(navigator, "serial", { value: tripwire("serial"), configurable: true });
      });
      page.on("pageerror", (error) => guard.pageErrors.push(`${error.name}: ${error.message}`));
      page.on("console", (message) => {
        if (message.type() === "error") guard.consoleErrors.push(message.text());
      });
      await use(guard);
      expect(guard.blocked, "requests that tried to leave the machine").toEqual([]);
      const tripped = page.isClosed() ? [] : await page.evaluate(() => (window as unknown as { __tripped?: string[] }).__tripped ?? []).catch(() => []);
      expect(tripped, "calls that reached the real WebUSB / Web Serial").toEqual([]);
      const unexpected = guard.pageErrors.filter((e) => !guard.allowPageErrors.some((re) => re.test(e)));
      expect(unexpected, "uncaught page errors").toEqual([]);
    },
    { auto: true },
  ],
});

export { expect };

/** Opens an app page and waits until its script has installed its handlers (body[data-ready]). */
export async function openApp(page: Page, path: string): Promise<void> {
  await page.goto(path);
  await page.locator("body[data-ready=true]").waitFor({ timeout: 30_000 });
}

/** The joint table's positions, rad, by joint name. */
export async function tablePositions(page: Page): Promise<Record<string, number>> {
  return page.$$eval("#joints tr", (rows) =>
    Object.fromEntries(rows.map((r) => [(r as HTMLElement).dataset.joint!, Number((r.querySelector(".pos") as HTMLElement).dataset.rad)])),
  );
}

/** A mock hook (window.__mock) called in the page. */
export function hook<T = unknown>(page: Page, name: string, ...args: unknown[]): Promise<T> {
  return page.evaluate(
    ([n, a]) => {
      const path = (n as string).split(".");
      let target: any = (window as any).__mock;
      for (const p of path.slice(0, -1)) target = target[p];
      return target[path.at(-1)!](...(a as unknown[]));
    },
    [name, args] as const,
  ) as Promise<T>;
}

export const maxAbsDiff = (a: readonly number[], b: readonly number[]) => Math.max(...a.map((v, i) => Math.abs(v - b[i]!)));

/** Moves a range input as a hand on the slider would (fires its input event). */
export async function slide(page: Page, selector: string, value: number): Promise<void> {
  await page.locator(selector).evaluate((el, v) => {
    const input = el as HTMLInputElement;
    input.value = String(v);
    input.dispatchEvent(new Event("input", { bubbles: true }));
  }, value);
}

/** Samples `hook` every `everyMs` for `ms`, in the page, with the page's clock (s). */
export function sample(page: Page, op: string, ms: number, everyMs = 20, ...args: unknown[]): Promise<{ t: number; v: number[] }[]> {
  return page.evaluate(
    async ([o, total, every, a]) => {
      const out: { t: number; v: number[] }[] = [];
      const until = performance.now() + (total as number);
      while (performance.now() < until) {
        const v = (await (window as any).__mock[o as string](...(a as unknown[]))) as number[];
        out.push({ t: performance.now() / 1000, v });
        await new Promise((r) => setTimeout(r, every as number));
      }
      return out;
    },
    [op, ms, everyMs, args] as const,
  );
}

/** The fastest a joint moved over any `window` s of the samples, rad/s (a window long enough to keep timer jitter out). */
export function peakSpeed(samples: { t: number; v: number[] }[], joint: number, window = 0.1): number {
  let peak = 0;
  let from = samples[0]!;
  for (const s of samples.slice(1)) {
    if (s.t - from.t < window) continue;
    peak = Math.max(peak, Math.abs(s.v[joint]! - from.v[joint]!) / (s.t - from.t));
    from = s;
  }
  return peak;
}

/** Width and height from a JPEG's start-of-frame marker. */
export function jpegSize(bytes: Buffer): { width: number; height: number } | null {
  let i = 2;
  while (i + 9 < bytes.length) {
    if (bytes[i] !== 0xff) return null;
    const marker = bytes[i + 1]!;
    const length = bytes.readUInt16BE(i + 2);
    if (marker >= 0xc0 && marker <= 0xcf && ![0xc4, 0xc8, 0xcc].includes(marker)) {
      return { height: bytes.readUInt16BE(i + 5), width: bytes.readUInt16BE(i + 7) };
    }
    i += 2 + length;
  }
  return null;
}
