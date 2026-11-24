// tests/unit/test_testing.py (shared by both arm SDKs), run on the Maker Arm's fakes.
import { testingSuite } from "../../arm-core/test/shared/testing.suite.js";
import { simulatedGroup } from "@makermods/maker-arm/testing";
import { FAKES } from "./fakes.js";

testingSuite(FAKES, simulatedGroup);
