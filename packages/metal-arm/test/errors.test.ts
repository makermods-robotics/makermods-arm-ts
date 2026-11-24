// tests/unit/test_errors.py (shared by both arm SDKs), run on the Metal Arm's fakes.
import { errorsSuite } from "../../arm-core/test/shared/errors.suite.js";
import { FAKES } from "./fakes.js";

errorsSuite(FAKES);
