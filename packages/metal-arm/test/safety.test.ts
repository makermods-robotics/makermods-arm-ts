import { profileTypeSuite, safetySuite } from "../../arm-core/test/shared/safety.suite.js";
import { FAKES } from "./fakes.js";

safetySuite(FAKES);
profileTypeSuite(FAKES);
