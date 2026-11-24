import { type Arm, ArmState, GsUsbBackend, requestAdapter } from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import { measureLoop, type Timing } from "./timing.ts";

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const status = $("status");
const buttons = ["enable", "wave", "rest", "estop"].map((id) => $<HTMLButtonElement>(id));
let arm: Arm | null = null;

const deg = (rad: number | null) => (rad === null ? "-" : ((rad * 180) / Math.PI).toFixed(1));

function show(): void {
  if (!arm) return;
  const s = arm.status();
  const rows = Object.entries(s.joints).map(([name, j]) =>
    `${name.padEnd(14)} ${deg(j.position).padStart(8)} deg  ${(j.temperature ?? NaN).toFixed(0).padStart(3)} C  ${j.fault ?? "ok"}`);
  status.textContent = [`state: ${s.state}${s.faultReason ? ` (${s.faultReason})` : ""}`, ...rows].join("\n");
}

async function run(action: () => Promise<unknown>): Promise<void> {
  try {
    await action();
  } catch (e) {
    status.textContent = `${e instanceof Error ? e.message : e}`;
    return;
  }
  show();
}

$("connect").onclick = () => run(async () => {
  const device = await requestAdapter(); // must run inside the click
  if (!device) return;
  const create = $<HTMLSelectElement>("product").value === "maker" ? createMaker : createMetal;
  arm = create(new GsUsbBackend(device));
  await arm.connect();
  for (const b of buttons) b.disabled = false;
  setInterval(() => arm?.state !== ArmState.DISCONNECTED && show(), 250);
});

$("enable").onclick = () => run(() => arm!.enable());

$("wave").onclick = () => run(async () => {
  const a = arm!;
  const i = a.jointNames.indexOf("wrist_roll");
  const start = a.getJointPositions();
  const t0 = a.clock.now();
  for (let t = 0; t < 6 && a.state === ArmState.ENABLED; t = a.clock.now() - t0) {
    const q = [...start];
    q[i] = start[i]! + (10 * Math.PI / 180) * Math.sin((2 * Math.PI * t) / 3);
    a.setJointTargets(q); // the arm's own loop rate-limits and sends the frames
    await a.clock.sleep(0.02);
  }
  await a.moveTo(start, { timeout: 2 });
});

$("rest").onclick = () => run(async () => {
  if (!(await arm!.stop())) throw new Error("return to rest not verified: the arm keeps holding. Support it, then E-stop.");
});

$("estop").onclick = () => run(() => arm!.estop());

// Keep the arm from being left energized by a closed tab: stop sending and release.
window.addEventListener("pagehide", () => void arm?.estop());

async function timing(seconds: number): Promise<Timing[]> {
  const out: Timing[] = [];
  for (const product of ["metal", "maker"] as const) {
    out.push(await measureLoop(product, "page", seconds));
    const worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
    out.push(await new Promise<Timing>((resolve, reject) => {
      worker.onmessage = (e) => ("error" in e.data ? reject(new Error(e.data.error)) : resolve(e.data));
      worker.onerror = (e) => reject(new Error(e.message));
      worker.postMessage({ product, seconds });
    }));
    worker.terminate();
  }
  return out;
}

const format = (rows: Timing[]) => rows.map((r) =>
  `${r.product.padEnd(6)} ${r.where.padEnd(7)} target ${r.targetHz} Hz, got ${r.achievedHz.toFixed(1)} Hz; `
  + `period median ${r.medianMs.toFixed(2)} ms, p99 ${r.p99Ms.toFixed(2)} ms, max ${r.maxMs.toFixed(2)} ms`).join("\n");

$("timing").onclick = async () => {
  $("result").textContent = "measuring...";
  $("result").textContent = format(await timing(3));
};

// ?smoke=1: run the timing without a click and leave the result for an automated check.
if (new URLSearchParams(location.search).has("smoke")) {
  timing(3).then(
    (rows) => {
      $("result").textContent = format(rows);
      (window as unknown as { smoke: unknown }).smoke = { ok: true, rows, usb: "usb" in navigator, serial: "serial" in navigator };
    },
    (e) => ((window as unknown as { smoke: unknown }).smoke = { ok: false, error: String(e) }),
  );
}
