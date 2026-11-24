# @makermods/arm-core

Shared core of the MakerMods arm SDKs: the arm state machine and safety logic, CAN over WebUSB
(candleLight / gs_usb adapters), kinematics, trajectory IK and gravity feed-forward.

Install an arm package (`@makermods/metal-arm`, `@makermods/maker-arm`) rather than this one; each
re-exports what an application needs.
