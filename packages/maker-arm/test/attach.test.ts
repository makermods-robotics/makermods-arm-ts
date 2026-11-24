// tests/unit/test_attach.py (shared by both arm SDKs), run on the Maker Arm's fakes.
import { attachSuite } from "../../arm-core/test/shared/attach.suite.js";
import { FAKES } from "./fakes.js";

attachSuite(FAKES);
