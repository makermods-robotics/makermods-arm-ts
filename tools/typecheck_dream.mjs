// Type-checks the mock playground, its browser test and the Dream test server (examples/mock-apps/tsconfig.dream.json),
// which need the Dream client checked out at ../dreamlabs-ts; without that checkout it skips them.
import { spawnSync } from "node:child_process";
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";

const dreamlabs = fileURLToPath(new URL("../../dreamlabs-ts/src/index.ts", import.meta.url));
if (!existsSync(dreamlabs)) {
  console.log(`Skipped the playground typecheck: the Dream client is not checked out (${dreamlabs} missing).`);
  process.exit(0);
}
const tsc = fileURLToPath(new URL("../node_modules/typescript/bin/tsc", import.meta.url));
const config = fileURLToPath(new URL("../examples/mock-apps/tsconfig.dream.json", import.meta.url));
const { status } = spawnSync(process.execPath, [tsc, "-p", config], { stdio: "inherit" });
process.exit(status ?? 1);
