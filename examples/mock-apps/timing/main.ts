import { $, errorText, say } from "../shared/ui.ts";
import { measureLoop, type Timing } from "./bench.ts";

const rows = $<HTMLTableSectionElement>("results");
const message = $("message");

function inWorker(product: "metal" | "maker", seconds: number, arms: 1 | 2): Promise<Timing[]> {
  const worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
  return new Promise<Timing[]>((resolve, reject) => {
    worker.onmessage = (e) => ("error" in e.data ? reject(new Error(e.data.error)) : resolve(e.data));
    worker.onerror = (e) => reject(new Error(e.message));
    worker.postMessage({ product, seconds, arms });
  }).finally(() => worker.terminate());
}

function show(r: Timing): void {
  const row = rows.insertRow();
  row.dataset.result = JSON.stringify(r);
  row.dataset.key = `${r.product}-${r.where}-${r.arm}`;
  for (const cell of [r.product, r.where, r.arm, r.targetHz, r.achievedHz.toFixed(1), r.medianMs.toFixed(2), r.p99Ms.toFixed(2), r.maxMs.toFixed(2), r.ticks]) {
    row.insertCell().textContent = String(cell);
  }
}

$<HTMLButtonElement>("measure").onclick = async () => {
  const button = $<HTMLButtonElement>("measure");
  button.disabled = true;
  rows.replaceChildren();
  const seconds = Number($<HTMLInputElement>("seconds").value);
  try {
    for (const product of ["metal", "maker"] as const) {
      for (const arms of [1, 2] as const) {
        const what = arms === 1 ? product : `two ${product} arms`;
        say(message, `Measuring ${what} in the page...`);
        (await measureLoop(product, "page", seconds, arms)).forEach(show);
        say(message, `Measuring ${what} in a worker...`);
        (await inWorker(product, seconds, arms)).forEach(show);
      }
    }
    say(message, "Done.");
  } catch (e) {
    say(message, errorText(e), "error");
  } finally {
    button.disabled = false;
  }
};
document.body.dataset.ready = "true";
