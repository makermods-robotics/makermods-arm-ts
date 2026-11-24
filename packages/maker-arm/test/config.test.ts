// tests/unit/test_config.py (shared by both arm SDKs), run on the Maker Arm's fakes.
import { configSuite } from "../../arm-core/test/shared/config.suite.js";
import { FAKES } from "./fakes.js";

configSuite(FAKES);
