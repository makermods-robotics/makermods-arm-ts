// tests/unit/test_bus.py (shared by both arm SDKs), run on the Metal Arm's fakes.
import { busSuite } from "../../arm-core/test/shared/bus.suite.js";
import { FAKES } from "./fakes.js";

busSuite(FAKES);
