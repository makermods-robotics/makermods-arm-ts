import { measureLoop } from "./bench.ts";

self.onmessage = async (event: MessageEvent<{ product: "metal" | "maker"; seconds: number; arms: 1 | 2 }>) => {
  try {
    self.postMessage(await measureLoop(event.data.product, "worker", event.data.seconds, event.data.arms));
  } catch (e) {
    self.postMessage({ error: e instanceof Error ? e.message : String(e) });
  }
};
