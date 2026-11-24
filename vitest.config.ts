import { fileURLToPath } from "node:url";
import { defineConfig } from "vitest/config";

const src = (pkg: string, file = "index.ts") => fileURLToPath(new URL(`./packages/${pkg}/src/${file}`, import.meta.url));
const dir = (pkg: string) => fileURLToPath(new URL(`./packages/${pkg}/src/`, import.meta.url));

// Tests run against the sources, so no build is needed between packages.
export default defineConfig({
  resolve: {
    alias: [
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)$/, replacement: dir("$1") + "index.ts" },
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)\/(.+)$/, replacement: dir("$1") + "$2.ts" },
    ],
  },
  test: {
    include: ["packages/*/test/**/*.test.ts"],
    testTimeout: 60_000,
  },
});
