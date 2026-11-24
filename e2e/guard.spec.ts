/** The network guard itself: a page's request to a Dream host is caught and aborted, and reported. */

import { expect, openApp, test } from "./fixtures.ts";

test("a request to anything but this machine is aborted and recorded", async ({ page, guard }) => {
  await openApp(page, "/timing/");
  const outcome = await page.evaluate(() =>
    fetch("https://gateway.dreamlabs.ai/v1/get_action", { method: "POST", body: "{}" }).then(
      (r) => `answered ${r.status}`,
      (e: Error) => `failed: ${e.name}`,
    ),
  );
  expect(outcome).toBe("failed: TypeError");
  expect(guard.blocked).toEqual(["https://gateway.dreamlabs.ai/v1/get_action"]);
  guard.blocked.length = 0; // expected here; any other test fails on it
});
