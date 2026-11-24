import "../shared/mock-gate.ts";
import {
  type Arm, ArmState, type CanBackend, GsUsbBackend, requestAdapter, requestSlcanPort, SlcanSerialBackend,
} from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import { $, deg, errorText, guarded, logPane, paneLogger, renderJoints, say } from "../shared/ui.ts";

const message = $("message");
const log = logPane($("log"));
const logger = paneLogger(log);
const jointSelect = $<HTMLSelectElement>("joint");
const buttons = {
  connect: $<HTMLButtonElement>("connect"),
  disconnect: $<HTMLButtonElement>("disconnect"),
  enable: $<HTMLButtonElement>("enable"),
  move: $<HTMLButtonElement>("move"),
  sine: $<HTMLButtonElement>("sine"),
  rest: $<HTMLButtonElement>("rest"),
  estop: $<HTMLButtonElement>("estop"),
};

let arm: Arm | null = null;
let device: USBDevice | null = null;
const linkSelect = $<HTMLSelectElement>("link");
// ?link=slcan preselects the SLCAN adapter.
if (new URLSearchParams(location.search).get("link") === "slcan") linkSelect.value = "slcan";
let busy = false;

/** Which buttons make sense now. */
function refreshButtons(): void {
  const state = arm?.state ?? ArmState.DISCONNECTED;
  const live = state === ArmState.ENABLED || state === ArmState.FAULT;
  buttons.connect.disabled = arm !== null;
  $<HTMLSelectElement>("product").disabled = arm !== null;
  linkSelect.disabled = arm !== null;
  buttons.disconnect.disabled = arm === null;
  buttons.enable.disabled = state !== ArmState.CONNECTED || busy;
  buttons.move.disabled = state !== ArmState.ENABLED || busy;
  buttons.sine.disabled = state !== ArmState.ENABLED || busy;
  jointSelect.disabled = arm === null;
  buttons.rest.disabled = !live || busy;
  buttons.estop.disabled = arm === null;
}

function render(): void {
  if (arm) {
    const s = arm.status();
    $("state").textContent = s.state;
    $("fault").textContent = s.faultReason ? `(${s.faultReason})` : "";
    renderJoints($<HTMLTableSectionElement>("joints"), s.joints);
  }
  refreshButtons();
}
setInterval(render, 100);

/** One operator action at a time; what it throws goes to the message line. */
async function action(work: () => Promise<unknown>): Promise<void> {
  busy = true;
  refreshButtons();
  const owner = arm;
  try {
    await work();
  } catch (e) {
    // An action cut short by an unplugged adapter: the unplug handler reports that.
    if (owner !== null && arm !== owner) log(`${errorText(e)} (the adapter was unplugged)`);
    else say(message, errorText(e), "error");
  } finally {
    busy = false;
    render();
  }
}

buttons.connect.onclick = () =>
  action(async () => {
    // Inside the click: the browser's chooser (USB devices, or serial ports).
    let backend: CanBackend;
    let name: string;
    let chosen: USBDevice | null = null;
    if (linkSelect.value === "slcan") {
      const port = await requestSlcanPort();
      if (!port) return say(message, "No adapter chosen.");
      const slcan = new SlcanSerialBackend(port, { logger });
      backend = slcan;
      name = `SLCAN adapter ${slcan.port}`;
    } else {
      chosen = await requestAdapter();
      if (!chosen) return say(message, "No adapter chosen.");
      backend = new GsUsbBackend(chosen, { logger });
      name = chosen.serialNumber ?? "adapter";
    }
    const create = $<HTMLSelectElement>("product").value === "maker" ? createMaker : createMetal;
    const candidate = create(backend, { logger });
    await candidate.connect();
    arm = candidate;
    device = chosen;
    jointSelect.replaceChildren(...arm.jointNames.map((n) => new Option(n, n)));
    jointSelect.value = arm.jointNames.includes("wrist_flex") ? "wrist_flex" : arm.jointNames[0]!;
    say(message, `Connected to ${name}: ${arm.jointNames.length} joints.`);
  });

buttons.disconnect.onclick = () =>
  action(async () => {
    const a = arm!;
    arm = null;
    device = null;
    await a.disconnect();
    $("state").textContent = "not connected";
    $("fault").textContent = "";
    say(message, "Disconnected.");
  });

buttons.enable.onclick = () => action(async () => (await arm!.enable(), say(message, "Holding the present pose.")));

buttons.move.onclick = () =>
  action(async () => {
    const a = arm!;
    const i = a.jointNames.indexOf(jointSelect.value);
    const q = a.getCommandedPositions();
    q[i] = q[i]! + Number($<HTMLInputElement>("delta").value);
    $("target").textContent = JSON.stringify(q.map((v) => Number(v.toFixed(4))));
    const arrived = await a.moveTo(q, { speed: 0.5 });
    say(message, arrived ? `Arrived: ${jointSelect.value} at ${deg(a.getJointPositions()[i]!)} deg.` : "The move did not arrive.", arrived ? "info" : "error");
  });

buttons.sine.onclick = () =>
  action(async () => {
    const a = arm!;
    const i = a.jointNames.indexOf("wrist_roll");
    const start = a.getCommandedPositions();
    const amplitude = (10 * Math.PI) / 180;
    const period = 2;
    const t0 = a.clock.now();
    let worst = 0;
    let lo = Infinity;
    let hi = -Infinity;
    for (let t = 0; t < 2 * period && a.state === ArmState.ENABLED; t = a.clock.now() - t0) {
      const q = [...start];
      q[i] = start[i]! + amplitude * Math.sin((2 * Math.PI * t) / period);
      a.setJointTargets(q); // the arm's own loop limits the rate and sends the frames
      await a.clock.sleep(0.02);
      const measured = a.getJointPositions()[i]! - start[i]!;
      lo = Math.min(lo, measured);
      hi = Math.max(hi, measured);
      // The motor follows the command one control period later: compare with where the sine was then, or with the
      // last command when the sleep overran a period (the motor cannot be past the newest target).
      const due = amplitude * Math.sin((2 * Math.PI * Math.min(t, a.clock.now() - t0 - 1 / a.config.controlRateHz)) / period);
      worst = Math.max(worst, Math.abs(measured - due));
    }
    await a.moveTo(start, { timeout: 2 });
    if (a.state !== ArmState.ENABLED) throw new Error(`sine stopped: the arm is ${a.state}`);
    $("sine").dataset.amplitude = String((hi - lo) / 2);
    $("sine").dataset.error = String(worst);
    say(message, `Sine done: amplitude ${deg((hi - lo) / 2)} deg, worst tracking error ${deg(worst, 2)} deg.`);
  });

buttons.rest.onclick = () =>
  action(async () => {
    if (await arm!.stop()) say(message, "At rest, torque released.");
    else say(message, "Return to rest not verified: the arm keeps holding. Support it, then E-stop.", "error");
  });

// Not through action(): an e-stop never waits for another operation.
buttons.estop.onclick = async () => {
  if (!(await guarded(message, () => arm!.estop()))) return;
  say(message, "E-stop: torque released on every motor.");
  render();
};

navigator.usb?.addEventListener("disconnect", (event) => {
  const gone = (event as USBConnectionEvent).device;
  if (!arm || gone !== device) return;
  const a = arm;
  arm = null;
  device = null;
  const t0 = performance.now();
  log("adapter unplugged");
  say(message, "Adapter unplugged: stopping.", "error");
  void (async () => {
    // Nothing reaches the motors any more; their own watchdogs drop torque. Close down the session cleanly.
    await a.estop().catch((e) => log(`estop after unplug: ${errorText(e)}`));
    await a.disconnect().catch((e) => log(`disconnect after unplug: ${errorText(e)}`));
    $("state").textContent = "adapter unplugged";
    $("fault").textContent = "";
    say(message, `Adapter unplugged; the session was closed in ${(performance.now() - t0).toFixed(0)} ms. Plug it back in and connect.`, "error");
    render();
  })();
});

// A closed tab must not leave the arm energized.
window.addEventListener("pagehide", () => void arm?.estop());
render();
document.body.dataset.ready = "true"; // the handlers are in place
