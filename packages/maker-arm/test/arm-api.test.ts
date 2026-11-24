// tests/unit/test_arm_api.py (shared by both arm SDKs), run on the Maker Arm's fakes.
import { armApiSuite } from "../../arm-core/test/shared/arm-api.suite.js";
import { FAKES } from "./fakes.js";

armApiSuite(FAKES);
