// tests/unit/test_arm.py (shared by both arm SDKs), run on the Metal Arm's fakes.
import { armSuite } from "../../arm-core/test/shared/arm.suite.js";
import { FAKES } from "./fakes.js";

armSuite(FAKES);
