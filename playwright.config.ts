import { createServer } from "node:net";
import { defineConfig, devices } from "@playwright/test";

/** A free TCP port on 127.0.0.1, picked once and inherited by the test workers. */
async function freePort(): Promise<number> {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => {
      const { port } = server.address() as { port: number };
      server.close(() => resolve(port));
    });
  });
}

process.env.MOCK_APPS_PORT ??= String(await freePort());
const port = Number(process.env.MOCK_APPS_PORT);

export default defineConfig({
  testDir: "e2e",
  // Several tests measure loop timing: one at a time.
  workers: 1,
  fullyParallel: false,
  forbidOnly: !!process.env.CI,
  retries: 0,
  timeout: 60_000,
  expect: { timeout: 10_000 },
  reporter: process.env.CI ? [["list"], ["html", { open: "never" }]] : "list",
  use: {
    baseURL: `http://127.0.0.1:${port}`,
    trace: "retain-on-failure",
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        launchOptions: {
          args: [
            // Cameras: Chromium's synthetic device, granted without a prompt; real cameras are never opened.
            "--use-fake-device-for-media-stream",
            "--use-fake-ui-for-media-stream",
            // No name resolves but the machine's own, so nothing can leave it even past the request routing.
            "--host-resolver-rules=MAP * ~NOTFOUND, EXCLUDE localhost, EXCLUDE 127.0.0.1",
          ],
        },
      },
    },
  ],
  webServer: {
    command: `npx vite --config examples/mock-apps/vite.config.ts --host 127.0.0.1 --port ${port} --strictPort`,
    url: `http://127.0.0.1:${port}/`,
    env: { MOCK_APPS_TEST: "1" },
    reuseExistingServer: false,
    stdout: "ignore",
    stderr: "pipe",
    timeout: 60_000,
  },
});
