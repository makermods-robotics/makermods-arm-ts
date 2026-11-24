import "../shared/mock-gate.ts";
import { ArmState } from "@makermods/arm-core";
import {
  createArm, LocalStorageCalibrationStore, requestPort, type So101Arm, type So101CalibrationSession,
} from "@makermods/so101";
import { $, deg, errorText, logPane, renderJoints, say } from "../shared/ui.ts";

const message = $("message");
const log = logPane($("log"));
const jointSelect = $<HTMLSelectElement>("joint");
const store = new LocalStorageCalibrationStore();
const ids = ["connect", "disconnect", "wiggle", "calibrate", "save", "cancel", "enable", "move", "rest", "estop"] as const;
const button = (id: (typeof ids)[number]) => $<HTMLButtonElement>(id);

let port: SerialPort | null = null;
let arm: So101Arm | null = null;
let session: So101CalibrationSession | null = null;
let busy = false;

function refreshButtons(): void {
  const state = arm?.state ?? ArmState.DISCONNECTED;
  const calibrating = session?.active ?? false;
  const idle = !busy && !calibrating;
  button("connect").disabled = arm !== null || busy;
  button("disconnect").disabled = arm === null || busy;
  button("wiggle").disabled = state !== ArmState.CONNECTED || !idle;
  button("calibrate").disabled = state !== ArmState.CONNECTED || !idle;
  button("save").disabled = session?.status().status !== "recording";
  button("cancel").disabled = !calibrating;
  button("enable").disabled = state !== ArmState.CONNECTED || !idle;
  button("move").disabled = state !== ArmState.ENABLED || busy;
  button("rest").disabled = !(state === ArmState.ENABLED || state === ArmState.FAULT) || busy;
  button("estop").disabled = arm === null;
}

function render(): void {
  if (arm) {
    const s = arm.status();
    $("state").textContent = s.state;
    $("fault").textContent = s.faultReason ? `(${s.faultReason})` : "";
    renderJoints($<HTMLTableSectionElement>("joints"), s.joints);
    if (!session?.active) {
      $("calibration").textContent = s.calibration.problem ?? `'${s.calibration.name}' loaded`;
    }
  }
  if (session) {
    const st = session.status();
    if (session.active) $("calibration").textContent = `${st.status}: ${st.message}`;
    const ranges = st.recordedRanges ?? {};
    $("ranges").replaceChildren(
      ...Object.entries(ranges).map(([name, r]) => {
        const row = document.createElement("tr");
        row.dataset.joint = name;
        row.innerHTML = `<th scope="row">${name}</th><td>${r.min}</td><td>${r.max}</td><td>${r.current}</td>`;
        return row;
      }),
    );
  }
  refreshButtons();
}
setInterval(render, 100);

async function action(work: () => Promise<unknown>): Promise<void> {
  busy = true;
  refreshButtons();
  try {
    await work();
  } catch (e) {
    say(message, errorText(e), "error");
  } finally {
    busy = false;
    render();
  }
}

button("connect").onclick = () =>
  action(async () => {
    port ??= await requestPort(); // inside the click: the browser's port chooser
    if (!port) return say(message, "No port chosen.");
    const candidate = createArm(port, { calibrationStore: store });
    await candidate.connect();
    arm = candidate;
    jointSelect.replaceChildren(...arm.jointNames.map((n) => new Option(n, n)));
    jointSelect.value = "shoulder_pan";
    say(message, `Connected: ${arm.jointNames.length} servos.${arm.calibrationProblem ? " Calibrate before enabling." : ""}`);
  });

button("disconnect").onclick = () =>
  action(async () => {
    const a = arm!;
    arm = null;
    await a.disconnect();
    $("state").textContent = "not connected";
    say(message, "Disconnected.");
  });

button("wiggle").onclick = () =>
  action(async () => {
    const result = await arm!.wiggle();
    say(message, result.message, result.success ? "info" : "error");
  });

button("calibrate").onclick = () => {
  try {
    session = arm!.calibrate({ method: "manual", onEvent: (kind, text) => log(`${kind}: ${text}`) });
    say(message, "Move every joint slowly end to end, then Save calibration.");
    void session.done.then(() => {
      const st = session!.status();
      if (st.status === "completed") say(message, `Calibration saved as '${st.name}'.`);
      else say(message, `Calibration ${st.status}: ${st.error ?? st.message}`, st.status === "error" ? "error" : "info");
      render();
    });
  } catch (e) {
    say(message, errorText(e), "error");
  }
  render();
};
button("save").onclick = () => {
  try {
    session!.complete();
  } catch (e) {
    say(message, errorText(e), "error");
  }
};
button("cancel").onclick = () => void session?.stop();

button("enable").onclick = () => action(async () => (await arm!.enable(), say(message, "Holding the present pose.")));

button("move").onclick = () =>
  action(async () => {
    const a = arm!;
    const i = a.jointNames.indexOf(jointSelect.value);
    const q = a.getCommandedPositions();
    q[i] = q[i]! + Number($<HTMLInputElement>("delta").value);
    $("target").textContent = JSON.stringify(q.map((v) => Number(v.toFixed(4))));
    const arrived = await a.moveTo(q, { speed: 0.5 });
    if (!arrived) throw new Error("The move did not arrive.");
    say(message, `Arrived: ${jointSelect.value} at ${deg(a.getJointPositions()[i]!)} deg.`);
  });

button("rest").onclick = () =>
  action(async () => {
    if (await arm!.stop()) say(message, "At rest, torque released.");
    else say(message, "Return to rest not completed: the arm keeps holding. Support it, then E-stop.", "error");
  });

button("estop").onclick = async () => {
  try {
    await arm!.estop();
    say(message, "E-stop: torque released on every servo.");
  } catch (e) {
    say(message, errorText(e), "error");
  }
  render();
};

window.addEventListener("pagehide", () => void arm?.estop());
render();
document.body.dataset.ready = "true";
