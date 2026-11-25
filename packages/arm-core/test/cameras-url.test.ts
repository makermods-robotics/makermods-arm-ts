// Port of the url: camera tests of the Python SDK's tests/unit/test_cameras.py. Its local HTTP server
// (tests/unit/mjpeg_server.py) becomes an in-memory fetch whose streams are paced by a VirtualClock,
// so nothing touches the network and every run is the same.
import { describe, expect, it } from "vitest";
import {
  Camera, type FetchLike, type FetchResponseLike, HttpCapture, isCameraId, JpegImage, type JpegDecoder, MAX_FRAME_BYTES,
  MAX_HEADERS, MAX_LINE_BYTES, parseCameraId, resolveCamera, URL_TIMEOUT_S, urlCameraInfo, urlOf,
} from "../src/cameras.js";
import { ConnectError, ValueError } from "../src/errors.js";
import { VirtualClock } from "../src/testing.js";

const enc = (text: string): Uint8Array => Uint8Array.from(text, (c) => c.charCodeAt(0));

function concat(...parts: Uint8Array[]): Uint8Array {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
}

/** A small JPEG-shaped picture: SOI, a start-of-frame segment of 64x48, one byte for its gray level, EOI. */
function jpeg(n: number, pad = 0): Uint8Array {
  const sof = [0xff, 0xc0, 0x00, 0x11, 0x08, 0x00, 48, 0x00, 64, 0x03, 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1];
  return concat(Uint8Array.from([0xff, 0xd8, ...sof, 0xff, 0xfe, 0x00, 0x03, n % 250]), new Uint8Array(pad),
    Uint8Array.of(0xff, 0xd9));
}

/** The gray level jpeg() put in a picture. */
const gray = (bytes: Uint8Array | null): number => bytes![25]!;

interface ServerOptions {
  fps?: number;
  dropAfter?: number | null;
  partHeaders?: string;
  /** Bytes of padding in every picture (to pass MAX_FRAME_BYTES). */
  pad?: number;
  /** Cut the end (EOI and the byte before it) off every odd-numbered picture. */
  truncate?: boolean;
}

/**
 * The Python test server as an in-memory fetch on `clock`: /stream.mjpg (multipart MJPEG at `fps`;
 * each picture's gray level is its number mod 250), /nolength.mjpg (parts without Content-Length),
 * /latest.jpg, /text, /manyheaders.mjpg, /stall.mjpg (headers, then nothing), and two streams whose
 * first part header never ends: /trickle.mjpg (a byte every 50 ms) and /longline.mjpg (100 kB, then a
 * byte every 50 ms). /repeatheaders.mjpg repeats one header name past MAX_HEADERS. `dropAfter` ends each
 * stream after that many frames; `truncate` cuts the end off every odd picture. Port 9 refuses connections.
 */
class FakeServer {
  readonly base = "http://127.0.0.1:8090";
  sent = 0;
  connections = 0;
  /** Streams cancelled or aborted by the client. */
  closed = 0;
  readonly fps: number;
  readonly dropAfter: number | null;
  readonly partHeaders: string;
  readonly pad: number;
  readonly truncate: boolean;

  constructor(readonly clock: VirtualClock, options: ServerOptions = {}) {
    this.fps = options.fps ?? 100.0;
    this.dropAfter = options.dropAfter ?? null;
    this.partHeaders = options.partHeaders ?? "";
    this.pad = options.pad ?? 0;
    this.truncate = options.truncate ?? false;
  }

  /** The n-th picture, its end cut off when `truncate` and n is odd. */
  #picture(n: number): Uint8Array {
    const data = jpeg(n, this.pad);
    return this.truncate && n % 2 === 1 ? data.subarray(0, -3) : data;
  }

  readonly fetch: FetchLike = async (url, init) => {
    if (url.startsWith("http://127.0.0.1:9/")) throw new TypeError("fetch failed (connection refused)");
    this.connections += 1;
    const path = url.slice(this.base.length);
    const headers = (h: Record<string, string>) => ({ get: (name: string) => h[name.toLowerCase()] ?? null });
    if (path === "/latest.jpg") {
      this.sent += 1;
      const data = this.#picture(this.sent);
      return { status: 200, headers: headers({ "content-type": "image/jpeg" }), body: this.#body(init.signal, [data]) };
    }
    if (path === "/text") {
      return { status: 200, headers: headers({ "content-type": "text/html" }), body: this.#body(init.signal, [enc("<html></html>")]) };
    }
    if (path === "/missing.mjpg") return { status: 404, headers: headers({ "content-type": "text/html" }), body: null };
    const multipart = headers({ "content-type": "multipart/x-mixed-replace; boundary=frame" });
    if (path === "/trickle.mjpg" || path === "/longline.mjpg") {
      const first = path === "/longline.mjpg" ? concat(enc("--frame\r\nX-Pad: "), new Uint8Array(100_000).fill(0x61))
        : enc("--frame\r\nX-Pad: ");
      return { status: 200, headers: multipart, body: this.#paced(init.signal, 0.05, (k) => (k === 0 ? first : enc("a"))) };
    }
    if (path === "/manyheaders.mjpg") {
      const head = Array.from({ length: MAX_HEADERS + 1 }, (_, i) => `X-H${i}: ${i}\r\n`).join("");
      return { status: 200, headers: multipart, body: this.#body(init.signal, [enc(`--frame\r\n${head}\r\n`)], false) };
    }
    if (path === "/repeatheaders.mjpg") {
      const head = Array.from({ length: MAX_HEADERS + 1 }, (_, i) => `X-H: ${i}\r\n`).join("");
      return { status: 200, headers: multipart, body: this.#body(init.signal, [enc(`--frame\r\n${head}\r\n`)], false) };
    }
    if (path === "/stall.mjpg") {
      return { status: 200, headers: multipart, body: this.#body(init.signal, [enc("--frame\r\n")], false) };
    }
    const withLength = path !== "/nolength.mjpg";
    return {
      status: 200, headers: multipart,
      body: this.#paced(init.signal, 1.0 / this.fps, (k) => {
        if (this.dropAfter !== null && k >= this.dropAfter) return null;
        this.sent += 1;
        const data = this.#picture(this.sent);
        let head = `--frame\r\nContent-Type: image/jpeg\r\n${this.partHeaders}`;
        if (withLength) head += `Content-Length: ${data.length}\r\n`;
        return concat(enc(`${head}\r\n`), data, enc("\r\n"));
      }),
    };
  };

  /** A body of `chunks`, then the end of the stream (or nothing more, `end` false). */
  #body(signal: AbortSignal, chunks: Uint8Array[], end = true): ReadableStream<Uint8Array> {
    return new ReadableStream<Uint8Array>({
      start: (controller) => {
        for (const c of chunks) controller.enqueue(c);
        if (end) controller.close();
        signal.addEventListener("abort", () => {
          this.closed += 1;
          try {
            controller.error(new DOMException("aborted", "AbortError"));
          } catch {
            // already closed
          }
        });
      },
      cancel: () => {
        this.closed += 1;
      },
    });
  }

  /** A body written every `period` s, `chunk(k)` for the k-th write; null ends the stream. */
  #paced(signal: AbortSignal, period: number, chunk: (k: number) => Uint8Array | null): ReadableStream<Uint8Array> {
    let stop = (): void => undefined;
    let done = false;
    const finish = () => {
      if (done) return;
      done = true;
      this.closed += 1;
      stop();
    };
    return new ReadableStream<Uint8Array>({
      start: (controller) => {
        let k = 0;
        const write = () => {
          if (done) return;
          const c = chunk(k++);
          if (c === null) {
            done = true;
            controller.close();
            return;
          }
          controller.enqueue(c);
          stop = this.clock.timer(period, write);
        };
        write();
        signal.addEventListener("abort", () => {
          finish();
          try {
            controller.error(new DOMException("aborted", "AbortError"));
          } catch {
            // already closed
          }
        });
      },
      cancel: finish,
    });
  }
}

function setup(options: ServerOptions = {}) {
  const clock = new VirtualClock({ start: 100 });
  const server = new FakeServer(clock, options);
  const wall = () => 1_700_000_000 + clock.now();
  const camera = (path: string, extra: { fps?: number; decode?: JpegDecoder } = {}) =>
    new Camera(`url:${server.base}${path}`, { clock, wallTime: wall, fetch: server.fetch, logger: quiet, ...extra });
  const capture = (path: string) => HttpCapture.open(`${server.base}${path}`, { clock, wallTime: wall, fetch: server.fetch });
  /** Waits (on the virtual clock) until `until()` holds or `limit` s passed. */
  const waitFor = async (until: () => boolean, limit = 2.0) => {
    const deadline = clock.now() + limit;
    while (!until() && clock.now() < deadline) await clock.sleep(0.02);
    return until();
  };
  return { clock, server, wall, camera, capture, waitFor };
}

const quiet = { info() {}, warn() {}, error() {} };

describe("url: camera ids", () => {
  it("url ids resolve and are recognized", async () => {
    expect(await resolveCamera("url:http://127.0.0.1:8090/front/stream.mjpg")).toBe("http://127.0.0.1:8090/front/stream.mjpg");
    expect(isCameraId("url:https://cam.local/x.mjpg") && isCameraId("usb:abc") && isCameraId("device:abc")).toBe(true);
    expect(isCameraId("url:ftp://x") || isCameraId("front")).toBe(false);
    expect(() => parseCameraId("url:ftp://x")).toThrow(ValueError);
    expect(() => new Camera("url:ftp://x")).toThrow(ValueError);
    expect(parseCameraId("url:https://h/s.mjpg")).toEqual({ kind: "url", url: "https://h/s.mjpg" });
    const info = urlCameraInfo("url:http://h/s.mjpg");
    expect([info.id, info.path, info.index, info.name]).toEqual(["url:http://h/s.mjpg", "http://h/s.mjpg", -1, "MJPEG over HTTP"]);
    expect([urlOf("url:http://h/"), urlOf("usb:x"), urlOf(3)]).toEqual(["http://h/", null, null]);
  });
});

describe("url: cameras", () => {
  it("a stream gives the newest frame, never a backlog", async () => {
    const { clock, server, wall, camera } = setup({ fps: 200.0 });
    // Decoding takes 20 ms, so about four JPEGs arrive per decode: all but the newest are skipped.
    const slow: JpegDecoder = async (bytes, ts) => {
      await clock.sleep(0.02);
      return new JpegImage(bytes, ts, 64, 48);
    };
    const cam = camera("/stream.mjpg", { decode: slow });
    await cam.open();
    try {
      for (let i = 0; i < 4; i++) {
        await clock.sleep(0.25); // a slow reader: about 50 frames arrive meanwhile
        const frame = cam.newest()!;
        expect(frame).not.toBeNull();
        expect(frame.ageS).toBeLessThan(0.1);
        // the frame shown is one of the last few sent, not the next one in a queue
        expect((((server.sent - gray(frame.jpeg)) % 250) + 250) % 250).toBeLessThanOrEqual(10);
        frame.close();
      }
      expect(cam.skipped).toBeGreaterThan(50);
      expect(cam.frames + cam.skipped).toBeGreaterThanOrEqual(server.sent - 10); // every JPEG decoded or skipped for a newer one
      const fresh = await cam.read(1.0);
      expect(fresh.ageS).toBeLessThan(0.1);
      expect(Math.abs(fresh.stamp - wall())).toBeLessThan(0.5);
      expect(fresh.ageEstimated).toBe(false);
      fresh.close();
    } finally {
      await cam.close();
    }
  });

  it("frames are stamped when they arrive, less the server's X-Frame-Age-Ms", async () => {
    const { clock, wall, camera } = setup({ fps: 50.0, partHeaders: "X-Frame-Age-Ms: 200.0\r\n" });
    const cam = camera("/stream.mjpg");
    await cam.open();
    try {
      await clock.sleep(0.2);
      const frame = cam.newest()!;
      expect(frame.ageS).toBeGreaterThanOrEqual(0.2); // the server says each frame was 200 ms old when sent
      expect(frame.ageS).toBeLessThan(0.35);
      frame.close();
      const stamps: number[] = [];
      for (let i = 0; i < 3; i++) {
        const f = await cam.read(1.0);
        stamps.push(f.stamp);
        f.close();
      }
      expect(stamps).toEqual([...stamps].sort((a, b) => a - b));
      expect(wall() - stamps[2]!).toBeGreaterThanOrEqual(0.2 - 1e-9);
    } finally {
      await cam.close();
    }
  });

  it("a stream without Content-Length", async () => {
    const { camera } = setup({ fps: 100.0 });
    const cam = camera("/nolength.mjpg");
    await cam.open();
    try {
      for (let i = 0; i < 5; i++) {
        const f = await cam.read(1.0);
        expect([f.width, f.height]).toEqual([64, 48]);
        expect(f.jpeg![0] === 0xff && f.jpeg![1] === 0xd8 && f.jpeg![f.jpeg!.length - 1] === 0xd9).toBe(true);
        f.close();
      }
    } finally {
      await cam.close();
    }
  });

  it("a stream that drops is reopened", async () => {
    const { server, camera, waitFor } = setup({ fps: 100.0, dropAfter: 10 });
    const cam = camera("/stream.mjpg");
    await cam.open();
    try {
      expect(await waitFor(() => cam.reopens >= 2, 10.0)).toBe(true);
      expect(server.connections).toBeGreaterThanOrEqual(3);
      const f = await cam.read(2.0); // frames keep coming after the reconnects
      expect(f.ageS).toBeLessThan(0.2);
      f.close();
    } finally {
      await cam.close();
    }
  });

  it("a single JPEG is polled", async () => {
    const { server, camera } = setup();
    const cam = camera("/latest.jpg", { fps: 20 });
    await cam.open();
    try {
      const a = await cam.read(1.0);
      const b = await cam.read(1.0);
      expect(gray(b.jpeg)).not.toBe(gray(a.jpeg));
      expect(b.stamp).toBeGreaterThan(a.stamp);
      expect(b.captured - a.captured).toBeGreaterThanOrEqual(0.05 - 1e-9); // at most 20 a second
      expect(server.connections).toBeGreaterThanOrEqual(2);
      a.close();
      b.close();
    } finally {
      await cam.close();
    }
  });

  it("a URL that is not a camera fails open", async () => {
    const { camera } = setup();
    await expect(camera("/text").open()).rejects.toThrow(/text\/html/);
    await expect(camera("/missing.mjpg").open()).rejects.toThrow(/HTTP Error 404/);
    const refused = new Camera("url:http://127.0.0.1:9/none.mjpg", { clock: new VirtualClock(), fetch: setup().server.fetch });
    const failed = refused.open();
    await expect(failed).rejects.toThrow(ConnectError);
    await expect(failed).rejects.toThrow(/cannot open/);
  });

  it.each(["/stream.mjpg", "/nolength.mjpg", "/latest.jpg"])("a frame larger than the limit stops the capture (%s)", async (path) => {
    const { capture, waitFor } = setup({ fps: 50.0, pad: MAX_FRAME_BYTES });
    const c = await capture(path);
    try {
      expect(await waitFor(() => !c.alive)).toBe(true);
      expect(c.received).toBe(0);
      expect(c.error).toContain("larger than");
    } finally {
      c.release();
    }
  });

  it("a header line longer than the limit stops the capture", async () => {
    const { capture, waitFor } = setup();
    const c = await capture("/longline.mjpg");
    try {
      expect(await waitFor(() => !c.alive)).toBe(true);
      expect(c.error).toContain(`longer than ${MAX_LINE_BYTES}`);
    } finally {
      c.release();
    }
  });

  it("a part with too many header lines stops the capture", async () => {
    const { capture, waitFor } = setup();
    const c = await capture("/manyheaders.mjpg");
    expect(await waitFor(() => !c.alive)).toBe(true);
    expect(c.error).toContain(`more than ${MAX_HEADERS} header lines`);
    c.release();
  });

  it("a part repeating one header name past the limit stops the capture", async () => {
    const { capture, waitFor } = setup();
    const c = await capture("/repeatheaders.mjpg");
    expect(await waitFor(() => !c.alive)).toBe(true);
    expect(c.error).toContain(`more than ${MAX_HEADERS} header lines`);
    c.release();
  });

  it.each(["/stream.mjpg", "/nolength.mjpg", "/latest.jpg"])("an incomplete JPEG is dropped, not published (%s)", async (path) => {
    const { capture, waitFor } = setup({ fps: 50.0, truncate: true });
    const c = await capture(path);
    try {
      expect(await waitFor(() => c.incomplete >= 3 && c.received >= 3)).toBe(true);
      for (let i = 0; i < 4; i++) {
        const bytes = await c.read(1.0);
        expect(bytes).not.toBeNull();
        expect(gray(bytes) % 2).toBe(0); // only the complete (even) pictures
        expect(bytes![bytes!.length - 2] === 0xff && bytes![bytes!.length - 1] === 0xd9).toBe(true);
      }
      expect(c.alive).toBe(true); // a bad frame is skipped; the stream goes on
      expect(c.error).toBeNull();
    } finally {
      c.release();
    }
  });

  it("a url: camera never shows an incomplete JPEG", async () => {
    const { camera } = setup({ fps: 50.0, truncate: true });
    const cam = camera("/stream.mjpg");
    await cam.open();
    try {
      for (let i = 0; i < 5; i++) {
        const f = await cam.read(1.0);
        expect(gray(f.jpeg) % 2).toBe(0);
        f.close();
      }
    } finally {
      await cam.close();
    }
  });

  it("a server that stops sending times out", async () => {
    const { clock, capture, waitFor } = setup();
    const c = await capture("/stall.mjpg");
    const start = clock.now();
    expect(await waitFor(() => !c.alive, URL_TIMEOUT_S + 1)).toBe(true);
    expect(clock.now() - start).toBeGreaterThanOrEqual(URL_TIMEOUT_S - 0.03);
    expect(c.error).toContain("timed out");
    c.release();
  });

  it("a connect that never answers times out", async () => {
    const clock = new VirtualClock();
    const hanging: FetchLike = (_url, init) => new Promise<FetchResponseLike>((_, reject) => {
      init.signal.addEventListener("abort", () => reject(new DOMException("aborted", "AbortError")));
    });
    await expect(HttpCapture.open("http://h/x.mjpg", { clock, fetch: hanging })).rejects.toThrow(/cannot open camera url:http:\/\/h\/x.mjpg: timed out/);
    expect(clock.now()).toBe(URL_TIMEOUT_S);
  });

  it("releasing a url: camera does not wait for a trickling server", async () => {
    const { clock, server, capture, waitFor } = setup();
    const c = await capture("/trickle.mjpg");
    await clock.sleep(0.3); // the reader is inside the endless header line
    expect(c.alive).toBe(true);
    const before = clock.now();
    c.release(); // returns at once
    expect(clock.now()).toBe(before);
    expect(await waitFor(() => !c.alive, 0.1)).toBe(true);
    expect(server.closed).toBe(1);
    expect(c.error).toBeNull(); // a release is not a failure
  });

  it("closing a url: camera does not wait for the server", async () => {
    const { clock, server, camera } = setup({ fps: 50.0 });
    const cam = camera("/stream.mjpg");
    await cam.open();
    const before = clock.now();
    await cam.close();
    expect(clock.now()).toBe(before);
    await clock.sleep(0.1);
    expect(server.closed).toBe(1);
    expect(cam.newest()).toBeNull();
  });

  it("snapshot gives the JPEG as served", async () => {
    const { camera } = setup({ fps: 50.0 });
    const cam = camera("/stream.mjpg");
    await cam.open();
    try {
      const bytes = await cam.snapshot();
      expect(bytes.length).toBeGreaterThan(0);
      expect(bytes[0] === 0xff && bytes[1] === 0xd8).toBe(true);
      await expect(cam.newest()!.toJpeg(0.85, { width: 32, height: 24 })).rejects.toThrow(/createImageBitmap/);
    } finally {
      await cam.close();
    }
  });
});
