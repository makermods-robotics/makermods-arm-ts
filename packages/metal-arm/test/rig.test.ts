// tests/unit/test_rig.py (shared by both arm SDKs), run on the Metal Arm's fakes.
import { rigSuite } from "../../arm-core/test/shared/rig.suite.js";
import { FAKES } from "./fakes.js";

rigSuite(FAKES);
