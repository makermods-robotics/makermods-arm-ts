import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";

const dir = (pkg: string) => fileURLToPath(new URL(`../../packages/${pkg}/src/`, import.meta.url));

// The example runs against the package sources, so no build is needed first.
export default defineConfig({
  root: fileURLToPath(new URL(".", import.meta.url)),
  resolve: {
    alias: [
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)$/, replacement: dir("$1") + "index.ts" },
      { find: /^@makermods\/(arm-core|metal-arm|maker-arm|so101)\/(.+)$/, replacement: dir("$1") + "$2.ts" },
    ],
  },
  worker: { format: "es" },
});
