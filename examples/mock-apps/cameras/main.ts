import { Camera, listCameras } from "@makermods/arm-core";
import { $, errorText, say } from "../shared/ui.ts";

const message = $("message");
const select = $<HTMLSelectElement>("camera");
const preview = $<HTMLVideoElement>("preview");
const button = (id: string) => $<HTMLButtonElement>(id);
let camera: Camera | null = null;
let reads = 0;

function refresh(): void {
  button("open").disabled = camera !== null || select.options.length === 0;
  button("close").disabled = camera === null;
  button("read").disabled = camera === null;
  button("jpeg").disabled = camera === null;
}

async function guard(work: () => Promise<void>): Promise<void> {
  try {
    await work();
  } catch (e) {
    say(message, errorText(e), "error");
  }
  refresh();
}

button("list").onclick = () =>
  guard(async () => {
    const cameras = await listCameras(); // never opens a device
    select.replaceChildren(...cameras.map((c) => new Option(c.name, c.id)));
    $("cameras").replaceChildren(
      ...cameras.map((c) => {
        const row = document.createElement("tr");
        row.innerHTML = `<td><code></code></td><td></td>`;
        row.cells[0]!.firstElementChild!.textContent = c.id;
        row.cells[1]!.textContent = c.name;
        return row;
      }),
    );
    say(message, `${cameras.length} camera(s).`);
  });

button("open").onclick = () =>
  guard(async () => {
    const cam = new Camera(select.value, { width: 640, height: 480, fps: 30 });
    await cam.open(); // resolves once the first frame arrived
    camera = cam;
    preview.srcObject = cam.stream as MediaStream;
    await preview.play();
    say(message, `Opened ${select.value}.`);
  });

button("close").onclick = () =>
  guard(async () => {
    const cam = camera!;
    camera = null;
    await cam.close();
    preview.srcObject = null;
    $("age").textContent = "-";
    say(message, "Closed.");
  });

button("read").onclick = () =>
  guard(async () => {
    const frame = await camera!.read();
    try {
      $("frame").textContent = `${frame.width}x${frame.height}, ${(frame.ageS * 1000).toFixed(0)} ms old`;
      $("frame").dataset.width = String(frame.width);
      $("frame").dataset.height = String(frame.height);
      $("reads").textContent = String(++reads);
    } finally {
      frame.close();
    }
  });

button("jpeg").onclick = () =>
  guard(async () => {
    const size = { width: Number($<HTMLInputElement>("jpeg-width").value), height: Number($<HTMLInputElement>("jpeg-height").value) };
    const frame = await camera!.read();
    let bytes: Uint8Array;
    try {
      bytes = await frame.toJpeg(0.85, size);
    } finally {
      frame.close();
    }
    const blob = new Blob([bytes as BlobPart], { type: "image/jpeg" });
    const shown = await createImageBitmap(blob);
    $<HTMLImageElement>("snapshot").src = URL.createObjectURL(blob);
    const info = $("jpeg-info");
    info.textContent = `${bytes.length} bytes, decodes as ${shown.width}x${shown.height}`;
    let binary = "";
    for (const b of bytes) binary += String.fromCharCode(b);
    info.dataset.base64 = btoa(binary);
  });

// The newest frame's age, as a monitor would show it.
setInterval(() => {
  const frame = camera?.newest();
  if (!frame) return;
  $("age").textContent = `${(frame.ageS * 1000).toFixed(0)} ms`;
  $("age").dataset.ms = String(frame.ageS * 1000);
  frame.close();
}, 100);

refresh();
document.body.dataset.ready = "true";
