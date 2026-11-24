import "../shared/mock-gate.ts";
import { adapterPort, requestAdapter } from "@makermods/arm-core";
import { $, guarded, logPane, renderJoints, say } from "../shared/ui.ts";
import type { Command, Reply } from "./protocol.ts";

const message = $("message");
const log = logPane($("log"));
const jointSelect = $<HTMLSelectElement>("joint");
const worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
let state = "not connected";
let pending: Command["cmd"] | null = null;

const ids = ["connect", "disconnect", "enable", "move", "rest", "estop"] as const;
const button = (id: (typeof ids)[number]) => $<HTMLButtonElement>(id);

function refreshButtons(): void {
  const connected = state !== "not connected";
  const live = state === "enabled" || state === "fault";
  button("connect").disabled = connected || pending !== null;
  button("disconnect").disabled = !connected || pending !== null;
  button("enable").disabled = state !== "connected" || pending !== null;
  button("move").disabled = state !== "enabled" || pending !== null;
  button("rest").disabled = !live || pending !== null;
  button("estop").disabled = !connected;
}

function send(command: Command): void {
  if (command.cmd !== "estop") pending = command.cmd;
  refreshButtons();
  worker.postMessage(command);
}

let updates = 0;
worker.onmessage = (event: MessageEvent<Reply>) => {
  const r = event.data;
  if (r.type === "log") return log(r.line);
  if (r.type === "done") {
    if (r.cmd === pending) pending = null;
    say(message, r.message, r.ok ? "info" : "error");
    if (r.target) $("target").textContent = JSON.stringify(r.target.map((v) => Number(v.toFixed(4))));
  } else {
    $("updates").textContent = String(++updates);
    state = r.status?.state ?? "not connected";
    $("state").textContent = state;
    $("fault").textContent = r.status?.faultReason ? `(${r.status.faultReason})` : "";
    if (r.status) renderJoints($<HTMLTableSectionElement>("joints"), r.status.joints);
    if (r.jointNames.length && jointSelect.options.length !== r.jointNames.length) {
      jointSelect.replaceChildren(...r.jointNames.map((n) => new Option(n, n)));
      jointSelect.value = "wrist_flex";
    }
  }
  refreshButtons();
};
worker.onerror = (event) => say(message, `worker error: ${event.message}`, "error");

button("connect").onclick = () =>
  guarded(message, async () => {
    const device = await requestAdapter(); // the chooser needs the page and the click
    if (!device) return say(message, "No adapter chosen.");
    send({ cmd: "connect", product: $<HTMLSelectElement>("product").value as "metal" | "maker", port: adapterPort(device) });
  });
button("disconnect").onclick = () => send({ cmd: "disconnect" });
button("enable").onclick = () => send({ cmd: "enable" });
button("move").onclick = () => send({ cmd: "move", joint: jointSelect.value, delta: Number($<HTMLInputElement>("delta").value) });
button("rest").onclick = () => send({ cmd: "rest" });
button("estop").onclick = () => send({ cmd: "estop" });

// The page's own responsiveness: pings answer at once, and the longest gap between 10 ms timer ticks is shown.
let pings = 0;
button("ping" as never).onclick = () => ($("pings").textContent = String(++pings));
let last = performance.now();
let stall = 0;
setInterval(() => {
  const now = performance.now();
  stall = Math.max(stall, now - last - 10);
  last = now;
  $("stall").textContent = stall.toFixed(0);
}, 10);
refreshButtons();
document.body.dataset.ready = "true"; // the handlers are in place
