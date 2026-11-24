import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";

const here = (path: string) => fileURLToPath(new URL(path, import.meta.url));
const pkg = (name: string) => here(`../../packages/${name}/src/`);
/** The Dream client's checkout next to this repository, when there is one. */
export const DREAMLABS = here("../../../dreamlabs-ts/src/");
const hasDreamlabs = existsSync(DREAMLABS);

const pages = [
  "index", "arm-console/index", "arm-worker/index", "timing/index", "so101/index", "playground/index", "teleop/index",
  "arm-follow/index", "rig/index", "cameras/index",
].filter((p) =>
  existsSync(here(`${p}.html`)),
);

// The apps run against the package sources (and the Dream client's, when checked out), so nothing is built first.
export default defineConfig({
  root: here("."),
  resolve: {
    alias: [
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)$/, replacement: pkg("$1") + "index.ts" },
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)\/(.+)$/, replacement: pkg("$1") + "$2.ts" },
      ...(hasDreamlabs
        ? [
            { find: /^@makermods\/dreamlabs$/, replacement: DREAMLABS + "index.ts" },
            { find: /^@makermods\/dreamlabs\/(ik|node)$/, replacement: DREAMLABS + "$1/index.ts" },
            { find: /^@makermods\/dreamlabs\/(.+)$/, replacement: DREAMLABS + "$1.ts" },
          ]
        : []),
    ],
  },
  define: { __HAS_DREAMLABS__: JSON.stringify(hasDreamlabs) },
  // Without the Dream client the playground's app is never loaded, so the dependency scan leaves it out.
  optimizeDeps: hasDreamlabs ? {} : { entries: ["**/*.html", "!playground/**"] },
  server: {
    // Under the browser tests (MOCK_APPS_TEST=1) a source edited meanwhile must not reload a page mid-test.
    ...(process.env.MOCK_APPS_TEST ? { hmr: false, watch: null } : {}),
    forwardConsole: false,
    fs: { allow: [here("../.."), ...(hasDreamlabs ? [DREAMLABS] : [])] },
  },
  worker: { format: "es" },
  build: {
    rollupOptions: {
      input: Object.fromEntries(pages.map((p) => [p, here(`${p}.html`)])),
      external: hasDreamlabs ? [] : [/^@makermods\/dreamlabs/],
    },
  },
});
