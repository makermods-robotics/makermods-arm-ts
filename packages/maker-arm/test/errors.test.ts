// tests/unit/test_errors.py (shared by both arm SDKs) on the Maker Arm's fakes, and test_product.py.
import { describe, expect, it } from "vitest";
import { defaultConfig, MAKER_ARM } from "@makermods/maker-arm";
import { errorsSuite } from "../../arm-core/test/shared/errors.suite.js";
import { FAKES } from "./fakes.js";

errorsSuite(FAKES);

describe("product", () => {
  it("identity", () => {
    // Python: (PACKAGE, CLI_NAME, DIST_NAME) == ("maker_arm", "maker-arm", "maker-arm"). The TS
    // product names its npm package; there is no CLI or Python distribution.
    expect(MAKER_ARM.packageName).toBe("@makermods/maker-arm");
    // Python: DEFAULT_PROFILE, DEFAULT_STAR_MAPPING and URDF files exist. The TS product carries
    // the profile and URDF as data (no files, no Star mapping).
    expect(MAKER_ARM.defaultProfile).toBeTruthy();
    expect(typeof MAKER_ARM.defaultProfile.urdf).toBe("string");
    expect((MAKER_ARM.defaultProfile.urdf as string).startsWith("<?xml")).toBe(true);
    expect(defaultConfig().nJoints).toBe(7);
  });
});
