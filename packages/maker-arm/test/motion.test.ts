// Motion on the shipped profile (moveEe planning and ownership, the return onto the support) and the
// control loop's timing, run on the Maker Arm's fakes.
import { motionSuite } from "../../arm-core/test/shared/motion.suite.js";
import { FAKES } from "./fakes.js";

motionSuite(FAKES);
