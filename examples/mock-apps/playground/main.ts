import "../shared/mock-gate.ts";
import { $ } from "../shared/ui.ts";

declare const __HAS_DREAMLABS__: boolean;

if (__HAS_DREAMLABS__) await import("./app.ts");
else $("message").textContent = "This app needs the Dream client (@makermods/dreamlabs): check out dreamlabs-ts next to this repository.";
document.body.dataset.ready = String(__HAS_DREAMLABS__);
