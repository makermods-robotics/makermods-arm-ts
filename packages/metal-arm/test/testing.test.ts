// tests/unit/test_testing.py (shared by both arm SDKs), run on the Metal Arm's fakes.
import { testingSuite } from "../../arm-core/test/shared/testing.suite.js";
import { simulatedGroup } from "@makermods/metal-arm/testing";
import { FAKES } from "./fakes.js";

testingSuite(FAKES, simulatedGroup);
