import { measureLoop } from "./timing.ts";

self.onmessage = async (event: MessageEvent<{ product: "maker" | "metal"; seconds: number }>) => {
  try {
    self.postMessage(await measureLoop(event.data.product, "worker", event.data.seconds));
  } catch (e) {
    self.postMessage({ error: String(e) });
  }
};
