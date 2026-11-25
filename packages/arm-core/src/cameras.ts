/**
 * Cameras in a browser: list them without opening them, and capture the newest frame in the
 * background, reopening a camera that ends or stops sending.
 *
 *     const cam = new Camera("label:Top Cam");
 *     await cam.open();                       // resolves once the first frame arrived
 *     const frame = await cam.read();         // the next new frame
 *     const jpeg = await frame.toJpeg(0.85, { width: 320, height: 176 });
 *     frame.close();                          // frames hold camera buffers: close each one
 *     await cam.close();
 *
 * Ids. The Python SDK names a camera by its USB port (`usb:<by-path name>`, Linux) or OpenCV index
 * (`index:N`). A browser sees neither USB paths nor device nodes: it has `deviceId` (an opaque id,
 * stable for this site while its browsing data is kept), `groupId` and `label` (the product name,
 * empty until the page has camera permission). So in a browser a camera is named
 *   - `device:<deviceId>`: exactly that device (the form listCameras() gives),
 *   - `label:<text>`: the only camera whose label is `<text>`, or else the only one whose label
 *     contains it (case-insensitive); identical cameras share a label, so name those by device,
 *   - `index:N`: the Nth video input in enumerateDevices() order (can change on replug).
 * `usb:` and `path:` ids parse (a Python rig file's `cameras:` entries load), but opening one fails
 * with a message saying to name that camera by device or label.
 *
 * `url:http://host:port/path` reads a camera another program serves over HTTP, as in the Python
 * SDK: a multipart MJPEG stream (`multipart/x-mixed-replace`, as from a camera server,
 * mjpg-streamer or an IP camera), or a single JPEG URL fetched again at most `fps` times a second.
 * It works in a page, a worker and Node (anywhere with `fetch` and web streams; a browser needs the
 * server to allow the page's origin). Header lines, header counts and frames are bounded
 * (MAX_LINE_BYTES, MAX_HEADERS, MAX_FRAME_BYTES): a stream past a bound fails and is reopened. A
 * JPEG without EOI at its end (a truncated part) is skipped and counted (HttpCapture.incomplete). Only
 * the newest JPEG is kept, so a slow reader gets the latest picture rather than a backlog; frames
 * are stamped when they arrive (less the server's `X-Frame-Age-Ms` part header when it sends one)
 * and keep the bytes as served (`Frame.jpeg`). A JPEG is decoded with createImageBitmap into a
 * VideoFrame where the runtime has both, else (Node) it stays a JpegImage. close() never waits on
 * the server.
 *
 * Capture. Each camera asks for 640x480 @ 30 fps (ideal/max constraints, so no camera is refused
 * for lacking the mode); one that cannot start there (two uncompressed streams on one USB2 hub) is
 * retried at 320x240 @ 15, then @ 10 fps, and stays at the mode that worked. Frames come from a
 * MediaStreamTrackProcessor where the browser has one (Chromium), else from a <video> element's
 * requestVideoFrameCallback (Firefox, Safari; page only). A track that ends (camera unplugged), or
 * delivers no frame for READ_FAILURE_S (frozen), is released and reopened after REOPEN_S, as the
 * Python SDK reopens a dead camera.
 *
 * Time. A frame's capture time comes from its VideoFrame timestamp (µs, on the system's monotonic
 * clock in Chromium; captureTime on the page's clock in the fallback), mapped to the camera's Clock
 * through an offset learnt as the smallest `now - timestamp` seen on the current stream. That
 * mapping can only make a frame look younger than it is by the fastest delivery delay (a few ms);
 * it is relearnt on every (re)opened stream. The first frame of a stream calibrates the offset by
 * itself and so reads as age ~0 whatever its real delay: its Frame has `ageEstimated` true, as has
 * a frame without a timestamp; from the second timestamped frame on the age is measured. A frame
 * whose timestamp does not advance past the previous one is the same capture again: it is dropped,
 * and neither counts as new for read() nor keeps the watchdog from declaring the camera frozen.
 * When a stream dies (ended, frozen, reader error) its newest frame is dropped, so newest() never
 * returns a picture from a dead stream. `captured` (Clock seconds) and `ageS` judge freshness;
 * `stamp` is the wall-clock capture time (s); a wall-clock step does not move ageS.
 *
 * Where it works. A page: everything. A Web Worker: no `navigator.mediaDevices` (getUserMedia is
 * page-only), so open the camera on the page; encodeJpeg() and Frame.toJpeg() work in a worker
 * (OffscreenCanvas), and in Chromium a MediaStreamTrackProcessor's readable can be transferred to a
 * worker and read there. Node: pass a fake `mediaDevices` (tests).
 */

import { type Clock, RealClock, Signal } from "./clock.js";
import { ConnectError, StateError, TimeoutError, ValueError } from "./errors.js";
import type { Logger } from "./logger.js";

/** s: this long without a frame is a dead (frozen) camera; it is released and reopened. */
export const READ_FAILURE_S = 1.0;
/** s: wait before reopening a dead camera, and between failed reopens. */
export const REOPEN_S = 1.0;
/** s: how often an open camera checks it is still receiving frames. */
export const WATCH_S = 0.1;
/** s: url: cameras, connect and read timeout. */
export const URL_TIMEOUT_S = 5.0;
/** url: cameras, longest header or boundary line. */
export const MAX_LINE_BYTES = 8192;
/** url: cameras, most header lines in one part. */
export const MAX_HEADERS = 64;
/** url: cameras, largest JPEG taken. */
export const MAX_FRAME_BYTES = 8 * 1024 * 1024;
/** s: url: cameras, how long one HttpCapture.read() waits for a new JPEG. */
export const URL_READ_WAIT_S = 0.1;
/** The schemes a url: camera may use. */
export const URL_SCHEMES: readonly string[] = Object.freeze(["http://", "https://"]);

// ── what the browser provides (injectable, so tests can fake it) ──

/** The parts of MediaDeviceInfo used here. */
export interface MediaDeviceInfoLike {
  readonly deviceId: string;
  readonly groupId: string;
  readonly kind: string;
  readonly label: string;
}

/** The parts of a MediaStreamTrack used here. */
export interface MediaStreamTrackLike {
  readonly readyState: "live" | "ended";
  readonly muted?: boolean;
  stop(): void;
  getSettings?(): { deviceId?: string; width?: number; height?: number; frameRate?: number };
  addEventListener(type: "ended" | "mute" | "unmute", listener: () => void): void;
  removeEventListener(type: "ended" | "mute" | "unmute", listener: () => void): void;
}

/** The parts of a MediaStream used here. */
export interface MediaStreamLike {
  getVideoTracks(): MediaStreamTrackLike[];
  getTracks(): MediaStreamTrackLike[];
}

/** The parts of navigator.mediaDevices used here. */
export interface MediaDevicesLike {
  enumerateDevices(): Promise<MediaDeviceInfoLike[]>;
  getUserMedia(constraints: MediaStreamConstraints): Promise<MediaStreamLike>;
}

/** The parts of a VideoFrame used here. A browser VideoFrame fits. */
export interface VideoFrameLike {
  /** µs; the capture time on the browser's clock. */
  readonly timestamp: number | null;
  readonly displayWidth: number;
  readonly displayHeight: number;
  clone(): VideoFrameLike;
  close(): void;
}

/** A frame stream for a capture track. */
export type FrameReader = (track: MediaStreamTrackLike) => ReadableStream<VideoFrameLike>;

/** The parts of an OffscreenCanvas toJpeg uses. */
export interface CanvasLike {
  getContext(kind: "2d"): { drawImage(image: never, dx: number, dy: number, dw: number, dh: number): void } | null;
  convertToBlob(options: { type: string; quality?: number }): Promise<{ type: string; arrayBuffer(): Promise<ArrayBuffer> }>;
}

/** A canvas of the given size (default: an OffscreenCanvas). */
export type CanvasFactory = (width: number, height: number) => CanvasLike;

/** The parts of a fetch() Response a url: camera uses. */
export interface FetchResponseLike {
  readonly status: number;
  readonly headers: { get(name: string): string | null };
  readonly body: ReadableStream<Uint8Array> | null;
}

/** fetch() as a url: camera calls it. */
export type FetchLike = (url: string, init: { headers: Record<string, string>; signal: AbortSignal; cache: "no-store" }) =>
  Promise<FetchResponseLike>;

/** A JPEG as a picture stamped `timestampUs`, or null when it does not decode. */
export type JpegDecoder = (jpeg: Uint8Array, timestampUs: number) => Promise<VideoFrameLike | null>;

function navigatorMediaDevices(): MediaDevicesLike {
  const md = (globalThis as { navigator?: { mediaDevices?: MediaDevicesLike } }).navigator?.mediaDevices;
  if (!md) {
    throw new ConnectError("cameras need navigator.mediaDevices: a page on https or localhost (a Web Worker has none; " +
      "open the camera on the page)");
  }
  return md;
}

const offscreenCanvas: CanvasFactory = (width, height) => {
  const C = (globalThis as { OffscreenCanvas?: new (w: number, h: number) => CanvasLike }).OffscreenCanvas;
  if (!C) throw new ConnectError("JPEG encoding needs OffscreenCanvas (a browser page or worker)");
  return new C(width, height);
};

/** The JPEG bytes of `image`, scaled to `size` (default: its own size), through a canvas. */
export async function encodeJpeg(image: VideoFrameLike, quality = 0.85, size?: { width: number; height: number } | null,
                                 canvas: CanvasFactory = offscreenCanvas): Promise<Uint8Array> {
  const width = size?.width ?? image.displayWidth;
  const height = size?.height ?? image.displayHeight;
  if (image instanceof JpegImage) {
    // Undecoded (no createImageBitmap here): the JPEG as served, which only fits its own size.
    if (width === image.displayWidth && height === image.displayHeight) return image.jpeg.slice();
    throw new ConnectError("scaling a url: camera's JPEG needs createImageBitmap and a canvas (a browser page or worker)");
  }
  const c = canvas(width, height);
  const ctx = c.getContext("2d");
  if (!ctx) throw new ConnectError("no 2d canvas context for JPEG encoding");
  ctx.drawImage(image as never, 0, 0, width, height);
  const blob = await c.convertToBlob({ type: "image/jpeg", quality });
  if (blob.type !== "image/jpeg") throw new ConnectError(`JPEG encoding unavailable (got ${blob.type || "nothing"})`);
  return new Uint8Array(await blob.arrayBuffer());
}

// ── discovery ──

/** A camera as listCameras() describes it. */
export interface CameraInfo {
  /** `device:<deviceId>`, or `index:N` before the page has camera permission (no deviceId yet). */
  id: string;
  /** Position among the video inputs. */
  index: number;
  /** The label (empty before permission: `camera N`). */
  name: string;
  /** The URL of a url: camera; null otherwise (Python: /dev/videoN; a browser has no device path). */
  path: string | null;
  deviceId: string;
  groupId: string;
}

/** Video inputs present now. Never opens a device; labels and ids are empty until the page has camera permission. */
export async function listCameras(mediaDevices: MediaDevicesLike = navigatorMediaDevices()): Promise<CameraInfo[]> {
  const inputs = (await mediaDevices.enumerateDevices()).filter((d) => d.kind === "videoinput");
  return inputs.map((d, index) => ({
    id: d.deviceId ? `device:${d.deviceId}` : `index:${index}`,
    index,
    name: d.label || `camera ${index}`,
    path: null,
    deviceId: d.deviceId,
    groupId: d.groupId,
  }));
}

/** A parsed camera id. */
export type CameraId =
  | { kind: "device"; deviceId: string }
  | { kind: "label"; label: string }
  | { kind: "index"; index: number }
  | { kind: "usb" | "path"; path: string }
  | { kind: "url"; url: string };

/** The URL a `url:` id names, or null for any other id. */
export function urlOf(cameraId: string | number): string | null {
  return typeof cameraId === "string" && cameraId.startsWith("url:") ? cameraId.slice(4) : null;
}

/** Is `text` a camera id (rather than a rig camera name)? */
export function isCameraId(text: string): boolean {
  const url = urlOf(text);
  if (url !== null) return URL_SCHEMES.some((scheme) => url.startsWith(scheme));
  return ["usb", "path", "index", "device", "label"].includes(text.split(":", 1)[0]!);
}

/** A CameraInfo for a `url:` id (nothing is fetched). */
export function urlCameraInfo(cameraId: string): CameraInfo {
  return { id: cameraId, index: -1, name: "MJPEG over HTTP", path: urlOf(cameraId), deviceId: "", groupId: "" };
}

/** Parses an id (`device:`, `label:`, `index:N`, a number, `url:http(s)://...`, or the Python SDK's `usb:` / `path:`). */
export function parseCameraId(id: string | number): CameraId {
  if (typeof id === "number") {
    if (!Number.isInteger(id) || id < 0) throw new ValueError(`camera index must be a non-negative integer, got ${id}`);
    return { kind: "index", index: id };
  }
  const colon = id.indexOf(":");
  const prefix = colon < 0 ? "" : id.slice(0, colon);
  const rest = colon < 0 ? "" : id.slice(colon + 1);
  if (prefix === "index" && /^\d+$/.test(rest)) return { kind: "index", index: Number(rest) };
  if (prefix === "device" && rest) return { kind: "device", deviceId: rest };
  if (prefix === "label" && rest) return { kind: "label", label: rest };
  if ((prefix === "usb" || prefix === "path") && rest) return { kind: prefix, path: rest };
  if (prefix === "url" && URL_SCHEMES.some((scheme) => rest.startsWith(scheme))) return { kind: "url", url: rest };
  throw new ValueError(`unknown camera id '${id}': expected device:<deviceId>, label:<text>, index:N, usb:<by-path>, ` +
    "path:<by-path> or url:http://...");
}

async function videoInputs(md: MediaDevicesLike): Promise<MediaDeviceInfoLike[]> {
  return (await md.enumerateDevices()).filter((d) => d.kind === "videoinput");
}

/**
 * The deviceId an id names among the present cameras, or a url: camera's URL. A label or index
 * needs the page's camera permission: when labels are still hidden, one getUserMedia({video: true})
 * asks for it (its stream is stopped at once) and the cameras are listed again.
 */
export async function resolveCamera(id: string | number, mediaDevices?: MediaDevicesLike): Promise<string> {
  const parsed = parseCameraId(id);
  const shown = typeof id === "number" ? `index:${id}` : id;
  if (parsed.kind === "url") return parsed.url;
  if (parsed.kind === "usb" || parsed.kind === "path") {
    throw new ConnectError(`camera ${shown} names a USB port, which a browser cannot see: name it device:<deviceId> or ` +
      "label:<text> (listCameras() lists both)");
  }
  if (parsed.kind === "device") return parsed.deviceId;
  mediaDevices ??= navigatorMediaDevices();
  let inputs = await videoInputs(mediaDevices);
  if (inputs.some((d) => !d.label || !d.deviceId)) {
    const stream = await mediaDevices.getUserMedia({ video: true, audio: false });
    for (const t of stream.getTracks()) t.stop();
    inputs = await videoInputs(mediaDevices);
  }
  if (parsed.kind === "index") {
    const d = inputs[parsed.index];
    if (!d?.deviceId) throw new ConnectError(`camera ${shown} is not connected (${inputs.length} camera(s) present)`);
    return d.deviceId;
  }
  if (parsed.kind !== "label") throw new ValueError(`unknown camera id '${shown}'`);
  const label = parsed.label;
  const exact = inputs.filter((d) => d.label === label);
  const lower = label.toLowerCase();
  const matches = exact.length ? exact : inputs.filter((d) => d.label.toLowerCase().includes(lower));
  if (matches.length === 1) return matches[0]!.deviceId;
  if (!matches.length) throw new ConnectError(`camera ${shown} is not connected`);
  throw new ConnectError(`several cameras match ${shown} (${matches.map((d) => d.label).join(", ")}); ` +
    "name one as device:<deviceId>");
}

// ── HTTP cameras ──

const LF = 0x0a;
const DASH = 0x2d;

function bytesOf(text: string): Uint8Array {
  return Uint8Array.from(text, (c) => c.charCodeAt(0) & 0xff);
}

function latin1(bytes: Uint8Array): string {
  let out = "";
  for (const b of bytes) out += String.fromCharCode(b);
  return out;
}

/** Python's bytes.strip(): ASCII whitespace off both ends. */
function stripBytes(b: Uint8Array): Uint8Array {
  const space = (c: number) => c === 0x20 || (c >= 0x09 && c <= 0x0d);
  let i = 0;
  let j = b.length;
  while (i < j && space(b[i]!)) i++;
  while (j > i && space(b[j - 1]!)) j--;
  return b.subarray(i, j);
}

function startsWith(b: Uint8Array, prefix: Uint8Array): boolean {
  if (b.length < prefix.length) return false;
  for (let i = 0; i < prefix.length; i++) if (b[i] !== prefix[i]) return false;
  return true;
}

function sameBytes(a: Uint8Array, b: Uint8Array): boolean {
  return a.length === b.length && startsWith(a, b);
}

function concatBytes(chunks: readonly Uint8Array[], size: number): Uint8Array {
  const out = new Uint8Array(size);
  let at = 0;
  for (const c of chunks) {
    out.set(c, at);
    at += c.length;
  }
  return out;
}

/** Python's bytes.lower(): ASCII letters only. */
function asciiLower(text: string): string {
  return text.replace(/[A-Z]/g, (c) => c.toLowerCase());
}

const isJpeg = (b: Uint8Array): boolean => b.length >= 2 && b[0] === 0xff && b[1] === 0xd8;

/** A whole JPEG: SOI first and EOI last (CR, LF and NUL padding after it allowed), not a truncated one. */
function isCompleteJpeg(b: Uint8Array): boolean {
  let end = b.length;
  while (end > 4 && (b[end - 1] === 0x0d || b[end - 1] === LF || b[end - 1] === 0x00)) end -= 1;
  return end >= 4 && isJpeg(b) && b[end - 2] === 0xff && b[end - 1] === 0xd9;
}

/** The media type of a response (`multipart/x-mixed-replace`, `image/jpeg`; text/plain when unnamed), lower case. */
function contentType(headers: FetchResponseLike["headers"]): string {
  const kind = asciiLower((headers.get("content-type") ?? "").split(";", 1)[0]!.trim());
  return kind.includes("/") ? kind : "text/plain";
}

/** Bytes of a stream read as lines and runs (Python's buffered readline(limit) / read(n)). */
class ByteReader {
  readonly #next: () => Promise<Uint8Array | null>;
  #chunks: Uint8Array[] = [];
  #size = 0;
  #eof = false;

  constructor(next: () => Promise<Uint8Array | null>) {
    this.#next = next;
  }

  async #more(): Promise<boolean> {
    if (this.#eof) return false;
    const chunk = await this.#next();
    if (chunk === null) {
      this.#eof = true;
      return false;
    }
    if (chunk.length) {
      this.#chunks.push(chunk);
      this.#size += chunk.length;
    }
    return true;
  }

  #take(n: number): Uint8Array {
    const out = new Uint8Array(n);
    let at = 0;
    while (at < n) {
      const c = this.#chunks[0]!;
      const k = Math.min(c.length, n - at);
      out.set(c.subarray(0, k), at);
      at += k;
      if (k === c.length) this.#chunks.shift();
      else this.#chunks[0] = c.subarray(k);
    }
    this.#size -= n;
    return out;
  }

  /** Index of the first LF within the first `limit` buffered bytes, or -1. */
  #newline(limit: number): number {
    let base = 0;
    for (const c of this.#chunks) {
      const i = c.subarray(0, Math.max(0, limit - base)).indexOf(LF);
      if (i >= 0) return base + i;
      base += c.length;
      if (base >= limit) break;
    }
    return -1;
  }

  /** One line with its LF, cut at `limit` bytes; shorter at the end of the stream, empty after it. */
  async readline(limit: number): Promise<Uint8Array> {
    for (;;) {
      const i = this.#newline(limit);
      if (i >= 0) return this.#take(i + 1);
      if (this.#size >= limit) return this.#take(limit);
      if (!(await this.#more())) return this.#take(this.#size);
    }
  }

  /** `n` bytes, fewer at the end of the stream. */
  async read(n: number): Promise<Uint8Array> {
    while (this.#size < n && (await this.#more()));
    return this.#take(Math.min(n, this.#size));
  }
}

/** Options of HttpCapture.open(). */
export interface HttpCaptureOptions {
  /** A single-JPEG URL is fetched again at most this often a second (default 30). */
  fps?: number;
  /** s, connect and read timeout (default URL_TIMEOUT_S). */
  timeout?: number;
  /** Default: globalThis.fetch. */
  fetch?: FetchLike;
  /** Default: a RealClock. */
  clock?: Clock;
  /** Wall-clock seconds (default: Date.now() / 1000). */
  wallTime?: () => number;
}

/**
 * A `url:` camera's HTTP stream. A receive loop parses it and keeps only the newest JPEG; read()
 * takes it. When the stream ends or fails the loop stops (`alive` false, `error` says why) and
 * read() keeps returning null; Camera reopens it.
 */
export class HttpCapture {
  readonly url: string;
  readonly timeout: number;
  /** s between fetches of a single-JPEG URL (0: as fast as it answers). */
  readonly pollPeriod: number;
  /** Why the receive loop stopped, or null. */
  error: string | null = null;
  /** JPEGs received. */
  received = 0;
  /** JPEGs replaced by a newer one before read() took them. */
  dropped = 0;
  /** JPEGs skipped because they were incomplete (no EOI at the end): never given by read(). */
  incomplete = 0;
  /** [wall-clock s, Clock s] arrival of the JPEG read() returned last (less the server's X-Frame-Age-Ms). */
  lastTimes: [number, number] | null = null;

  readonly #fetch: FetchLike;
  readonly #clock: Clock;
  readonly #wall: () => number;
  readonly #signal = new Signal();
  #jpeg: Uint8Array | null = null;
  #times: [number, number] = [0, 0];
  #seq = 0;
  #taken = 0;
  #closed = false;
  #alive = true;
  #abort: AbortController | null = null;
  #reader: ReadableStreamDefaultReader<Uint8Array> | null = null;

  private constructor(url: string, options: HttpCaptureOptions) {
    const fps = options.fps ?? 30.0;
    this.url = url;
    this.timeout = options.timeout ?? URL_TIMEOUT_S;
    this.pollPeriod = fps && fps > 0 ? 1.0 / fps : 0.0;
    this.#clock = options.clock ?? new RealClock();
    this.#wall = options.wallTime ?? (() => Date.now() / 1000);
    const f = options.fetch ?? (globalThis as { fetch?: FetchLike }).fetch?.bind(globalThis);
    if (!f) throw new ConnectError(`cannot open camera url:${url}: this runtime has no fetch()`);
    this.#fetch = f;
  }

  /** Connect (ConnectError when it fails or serves neither an MJPEG stream nor a JPEG) and start receiving. */
  static async open(url: string, options: HttpCaptureOptions = {}): Promise<HttpCapture> {
    const capture = new HttpCapture(url, options);
    const response = await capture.#connect();
    const multipart = contentType(response.headers).startsWith("multipart/");
    void capture.#receive(response, multipart);
    return capture;
  }

  /** Is the receive loop still running? */
  get alive(): boolean {
    return this.#alive;
  }

  async #connect(): Promise<FetchResponseLike> {
    const abort = new AbortController();
    this.#abort = abort;
    let cancel = (): void => undefined;
    const timedOut = new Promise<never>((_, reject) => {
      cancel = this.#clock.timer(this.timeout, () => {
        reject(new Error("timed out"));
        abort.abort();
      });
    });
    let response: FetchResponseLike;
    try {
      response = await Promise.race([
        this.#fetch(this.url, { headers: { "Cache-Control": "no-cache" }, signal: abort.signal, cache: "no-store" }), timedOut]);
    } catch (e) {
      abort.abort();
      throw new ConnectError(`cannot open camera url:${this.url}: ${errorText(e) || errorName(e)}`, { cause: e });
    } finally {
      cancel();
    }
    if (response.status >= 400) {
      abort.abort();
      throw new ConnectError(`cannot open camera url:${this.url}: HTTP Error ${response.status}`);
    }
    const kind = contentType(response.headers);
    if (!(kind.startsWith("multipart/") || kind.startsWith("image/")) || !response.body) {
      abort.abort();
      throw new ConnectError(`camera url:${this.url} serves ${response.body ? kind : "no body"}, not an MJPEG stream or a JPEG`);
    }
    return response;
  }

  /** The bytes of `response`, each read bounded by the timeout. */
  #bytes(response: FetchResponseLike): ByteReader {
    const reader = response.body!.getReader();
    this.#reader = reader;
    return new ByteReader(async () => {
      let cancel = (): void => undefined;
      const timedOut = new Promise<never>((_, reject) => {
        cancel = this.#clock.timer(this.timeout, () => reject(new Error("timed out")));
      });
      try {
        const { value, done } = await Promise.race([reader.read(), timedOut]);
        return done ? null : value;
      } finally {
        cancel();
      }
    });
  }

  /** Publishes a JPEG as the newest frame; bytes that are not a JPEG are ignored, an incomplete one is counted. */
  #offer(jpeg: Uint8Array, ageHeader: string | null | undefined): void {
    if (!isJpeg(jpeg)) return;
    if (isCompleteJpeg(jpeg)) this.#store(jpeg, ageHeader);
    else this.incomplete += 1;
  }

  #store(jpeg: Uint8Array, ageHeader: string | null | undefined): void {
    const now = this.#clock.now();
    const wall = this.#wall();
    let age = ageHeader == null ? 0.0 : Number(ageHeader) / 1000.0;
    if (Number.isNaN(age)) age = 0.0;
    age = Math.min(Math.max(age, 0.0), 10.0);
    if (this.#seq !== this.#taken) this.dropped += 1;
    this.#jpeg = jpeg;
    this.#times = [wall - age, now - age];
    this.#seq += 1;
    this.received += 1;
    this.#signal.notify();
  }

  async #receive(response: FetchResponseLike, multipart: boolean): Promise<void> {
    try {
      if (multipart) await this.#receiveParts(this.#bytes(response));
      else await this.#receivePolled(response);
    } catch (e) {
      // The stream ended or failed: read() reports no frames, Camera reopens.
      if (!this.#closed) this.error = `camera url:${this.url}: ${errorText(e) || errorName(e)}`;
    } finally {
      this.#abort?.abort();
      this.#reader?.cancel().catch(() => undefined);
      this.#alive = false;
      this.#signal.notify();
    }
  }

  async #line(bytes: ByteReader): Promise<Uint8Array> {
    const line = await bytes.readline(MAX_LINE_BYTES + 1);
    if (line.length > MAX_LINE_BYTES) throw new ValueError(`a header line longer than ${MAX_LINE_BYTES} bytes`);
    return line;
  }

  /**
   * multipart/x-mixed-replace: each part is headers, a blank line and one JPEG. Lines, headers and
   * frames are bounded (MAX_LINE_BYTES, MAX_HEADERS, MAX_FRAME_BYTES).
   */
  async #receiveParts(bytes: ByteReader): Promise<void> {
    const dashes = Uint8Array.of(DASH, DASH);
    let line = await this.#line(bytes);
    while (!this.#closed) {
      if (!line.length) throw new Error("the stream ended");
      if (!startsWith(line, dashes)) {
        line = await this.#line(bytes); // preamble, or the blank line after a part
        continue;
      }
      const boundary = stripBytes(line); // a closing boundary ("--frame--") is followed by the end of the stream
      const closing = Uint8Array.from([...boundary, DASH, DASH]);
      const headers = new Map<string, string>();
      let lines = 0; // physical header lines: a repeated name counts each time
      for (;;) {
        let h = await this.#line(bytes);
        if (!h.length) throw new Error("the stream ended inside a part");
        h = stripBytes(h);
        if (!h.length) break;
        if (lines++ >= MAX_HEADERS) throw new ValueError(`a part with more than ${MAX_HEADERS} header lines`);
        const colon = h.indexOf(0x3a);
        const name = colon < 0 ? h : h.subarray(0, colon);
        const value = colon < 0 ? new Uint8Array(0) : h.subarray(colon + 1);
        headers.set(asciiLower(latin1(stripBytes(name))), latin1(stripBytes(value)));
      }
      const length = headers.get("content-length");
      let jpeg: Uint8Array;
      if (length !== undefined && /^[0-9]+$/.test(length)) {
        const n = Number(length);
        if (n > MAX_FRAME_BYTES) throw new ValueError(`a frame of ${n} bytes, larger than ${MAX_FRAME_BYTES}`);
        jpeg = await bytes.read(n);
        if (jpeg.length < n) throw new Error("the stream ended inside a frame");
        line = await this.#line(bytes);
      } else {
        // No length: the JPEG runs up to the next boundary line (one that starts a line).
        const chunks: Uint8Array[] = [];
        let size = 0;
        let atLineStart = true;
        for (;;) {
          line = await bytes.readline(65536);
          if (!line.length) throw new Error("the stream ended inside a frame");
          if (atLineStart && (sameBytes(stripBytes(line), boundary) || startsWith(line, closing))) break;
          atLineStart = line[line.length - 1] === LF;
          size += line.length;
          if (size > MAX_FRAME_BYTES) throw new ValueError(`a frame larger than ${MAX_FRAME_BYTES} bytes`);
          chunks.push(line);
        }
        jpeg = concatBytes(chunks, size);
        if (jpeg.length >= 2 && jpeg[jpeg.length - 2] === 0x0d && jpeg[jpeg.length - 1] === LF) jpeg = jpeg.subarray(0, -2);
      }
      this.#offer(jpeg, headers.get("x-frame-age-ms"));
    }
  }

  /** A single-JPEG URL, fetched again at most `fps` times a second. */
  async #receivePolled(first: FetchResponseLike): Promise<void> {
    let response = first;
    while (!this.#closed) {
      const started = this.#clock.now();
      const jpeg = await this.#bytes(response).read(MAX_FRAME_BYTES + 1);
      this.#reader?.cancel().catch(() => undefined);
      if (jpeg.length > MAX_FRAME_BYTES) throw new ValueError(`a frame larger than ${MAX_FRAME_BYTES} bytes`);
      this.#offer(jpeg, response.headers.get("x-frame-age-ms"));
      const wait = this.pollPeriod - (this.#clock.now() - started);
      if (wait > 0) await this.#clock.sleep(wait);
      if (this.#closed) return;
      response = await this.#connect();
      if (this.#closed) return;
    }
  }

  /** The newest JPEG if it is newer than the last one returned, else null within `timeout` s. */
  async read(timeout = URL_READ_WAIT_S): Promise<Uint8Array | null> {
    const ok = await this.#signal.wait(() => this.#seq !== this.#taken || !this.#alive, timeout, this.#clock);
    if (!ok || this.#seq === this.#taken) return null;
    this.#taken = this.#seq;
    this.lastTimes = this.#times;
    return this.#jpeg;
  }

  /** Stop receiving. Aborts the request and returns at once: never waits on the server. */
  release(): void {
    this.#closed = true;
    this.#abort?.abort();
    this.#reader?.cancel().catch(() => undefined);
    this.#signal.notify();
  }
}

/** The pixel size in a JPEG's start-of-frame segment, or null when it has none. */
export function jpegSize(jpeg: Uint8Array): { width: number; height: number } | null {
  let i = 2;
  while (i + 9 < jpeg.length) {
    if (jpeg[i] !== 0xff) return null;
    const marker = jpeg[i + 1]!;
    if (marker === 0xff) {
      i += 1; // fill byte
      continue;
    }
    if (marker === 0xd8 || marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) {
      i += 2;
      continue;
    }
    const length = (jpeg[i + 2]! << 8) | jpeg[i + 3]!;
    if (marker >= 0xc0 && marker <= 0xcf && marker !== 0xc4 && marker !== 0xc8 && marker !== 0xcc) {
      return { height: (jpeg[i + 5]! << 8) | jpeg[i + 6]!, width: (jpeg[i + 7]! << 8) | jpeg[i + 8]! };
    }
    if (marker === 0xd9 || marker === 0xda || length < 2) return null;
    i += 2 + length;
  }
  return null;
}

/**
 * A JPEG that stays encoded (where the runtime has no createImageBitmap, e.g. Node): its size comes
 * from the JPEG header, `jpeg` holds the bytes, and Frame.toJpeg() returns them at that size.
 */
export class JpegImage implements VideoFrameLike {
  readonly jpeg: Uint8Array;
  readonly timestamp: number | null;
  readonly displayWidth: number;
  readonly displayHeight: number;

  constructor(jpeg: Uint8Array, timestamp: number | null, width: number, height: number) {
    this.jpeg = jpeg;
    this.timestamp = timestamp;
    this.displayWidth = width;
    this.displayHeight = height;
  }

  clone(): JpegImage {
    return new JpegImage(this.jpeg, this.timestamp, this.displayWidth, this.displayHeight);
  }

  close(): void {
    // nothing to release
  }
}

/**
 * The default JpegDecoder: createImageBitmap into a VideoFrame where the runtime has both (pages and
 * workers), else a JpegImage. Null for a JPEG that does not decode.
 */
export const decodeJpeg: JpegDecoder = async (jpeg, timestampUs) => {
  const g = globalThis as {
    createImageBitmap?: (blob: Blob) => Promise<{ close(): void }>;
    VideoFrame?: new (source: unknown, init: { timestamp: number }) => VideoFrameLike;
  };
  if (typeof g.createImageBitmap === "function" && typeof g.VideoFrame === "function") {
    try {
      const bitmap = await g.createImageBitmap(new Blob([jpeg as Uint8Array<ArrayBuffer>], { type: "image/jpeg" }));
      try {
        return new g.VideoFrame(bitmap, { timestamp: timestampUs });
      } finally {
        bitmap.close();
      }
    } catch {
      return null;
    }
  }
  const size = jpegSize(jpeg);
  return size ? new JpegImage(jpeg, timestampUs, size.width, size.height) : null;
};

// ── constraints ──

/** A capture mode: ideal size and frame rate, and the frame rate cap. */
export interface CaptureMode {
  width: number;
  height: number;
  frameRate: number;
  maxFrameRate: number;
}

/** Modes tried after the requested one fails to start (smaller than it only). */
export const FALLBACK_MODES: readonly CaptureMode[] = [
  { width: 320, height: 240, frameRate: 15, maxFrameRate: 30 },
  { width: 320, height: 240, frameRate: 10, maxFrameRate: 15 },
];

/** The modes a camera asking for width x height @ fps tries, in order. */
export function captureModes(width: number, height: number, fps: number): CaptureMode[] {
  const first = { width, height, frameRate: fps, maxFrameRate: fps };
  return [first, ...FALLBACK_MODES.filter((m) => m.width * m.height < width * height || m.frameRate < fps)];
}

/** getUserMedia constraints for `deviceId` (null: any camera) at `mode`. Only ideal / max, so no camera is refused. */
export function cameraConstraints(deviceId: string | null, mode: CaptureMode): MediaStreamConstraints {
  return {
    audio: false,
    video: {
      ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
      width: { ideal: mode.width },
      height: { ideal: mode.height },
      frameRate: { ideal: mode.frameRate, max: mode.maxFrameRate },
    },
  };
}

const errorName = (e: unknown): string => (e as { name?: string } | null)?.name ?? "Error";
const errorText = (e: unknown): string => (e instanceof Error ? e.message : (e as { message?: string } | null)?.message ?? String(e));

/** Errors no smaller mode can fix. */
const FATAL_START = new Set(["NotAllowedError", "SecurityError", "NotFoundError", "TypeError"]);

/** Why a camera did not start: the browser's error, and a hint. */
export function cameraStartError(id: string, error: unknown, otherLive = false): string {
  const name = errorName(error);
  const replug = "unplug it and plug it back in (or into another USB port), or close other apps using the camera";
  const hint = name === "NotAllowedError" || name === "SecurityError"
    ? "allow camera access for this site in the browser"
    : name === "NotFoundError" || name === "OverconstrainedError"
      ? "check it is plugged in"
      : otherLive
        ? "two cameras on one USB hub may exceed its bandwidth: plug one into another USB port, or close other apps using the camera"
        : `the camera or its USB hub may be at fault: ${replug}`;
  return `cannot open camera ${id} (${name}): the browser said "${errorText(error) || "no details"}"; ${hint}`;
}

// ── frame readers ──

type ProcessorClass = new (init: { track: MediaStreamTrackLike }) => { readable: ReadableStream<VideoFrameLike> };

/**
 * Frames of `track`: a MediaStreamTrackProcessor where the browser has one, else a <video>
 * element's requestVideoFrameCallback (page only; timestamps from the frames' captureTime).
 */
export const browserFrames: FrameReader = (track) => {
  const Processor = (globalThis as { MediaStreamTrackProcessor?: ProcessorClass }).MediaStreamTrackProcessor;
  if (typeof Processor === "function") return new Processor({ track }).readable;
  return videoElementFrames(track);
};

interface RvfcVideo {
  muted: boolean;
  playsInline: boolean;
  srcObject: unknown;
  play(): Promise<void>;
  pause(): void;
  requestVideoFrameCallback(cb: (now: number, metadata: { captureTime?: number; mediaTime?: number }) => void): number;
  cancelVideoFrameCallback(handle: number): void;
}

function videoElementFrames(track: MediaStreamTrackLike): ReadableStream<VideoFrameLike> {
  const g = globalThis as {
    document?: { createElement(tag: "video"): RvfcVideo };
    MediaStream?: new (tracks: unknown[]) => unknown;
    VideoFrame?: new (source: unknown, init: { timestamp: number }) => VideoFrameLike;
    performance?: { now(): number };
  };
  if (!g.document || !g.MediaStream || !g.VideoFrame) {
    throw new ConnectError("camera frames need MediaStreamTrackProcessor (Chromium), or a page with VideoFrame");
  }
  const video = g.document.createElement("video");
  if (typeof video.requestVideoFrameCallback !== "function") {
    throw new ConnectError("camera frames need MediaStreamTrackProcessor or requestVideoFrameCallback (update the browser)");
  }
  const VideoFrameClass = g.VideoFrame;
  let handle: number | null = null;
  let finished = false;
  /** The newest frame not yet handed to a read, and the read waiting for one (pull-based: nothing queues). */
  let pending: VideoFrameLike | null = null;
  let waiting: (() => void) | null = null;
  let onEnded: () => void = () => undefined;
  /** One idempotent teardown for cancel, an ended track and a failed play(). */
  const teardown = () => {
    if (finished) return;
    finished = true;
    if (handle !== null) video.cancelVideoFrameCallback(handle);
    handle = null;
    track.removeEventListener("ended", onEnded);
    video.pause();
    video.srcObject = null;
    pending?.close();
    pending = null;
    const w = waiting;
    waiting = null;
    w?.();
  };
  return new ReadableStream<VideoFrameLike>({
    start(controller) {
      video.muted = true;
      video.playsInline = true;
      video.srcObject = new g.MediaStream!([track]);
      const onFrame = (now: number, metadata: { captureTime?: number }) => {
        if (finished) return;
        const at = typeof metadata.captureTime === "number" && Number.isFinite(metadata.captureTime) ? metadata.captureTime : now;
        let frame: VideoFrameLike | null = null;
        try {
          frame = new VideoFrameClass(video, { timestamp: Math.round(at * 1000) });
        } catch {
          // no picture yet (the video has no current frame): skip this callback
        }
        if (frame) {
          if (waiting) {
            controller.enqueue(frame); // straight to the waiting read
            const w = waiting;
            waiting = null;
            w();
          } else {
            pending?.close();
            pending = frame;
          }
        }
        handle = video.requestVideoFrameCallback(onFrame);
      };
      handle = video.requestVideoFrameCallback(onFrame);
      onEnded = () => {
        if (finished) return;
        teardown();
        try {
          controller.close();
        } catch {
          // already closed or errored
        }
      };
      track.addEventListener("ended", onEnded);
      video.play().catch((e: unknown) => {
        if (finished) return;
        teardown();
        try {
          controller.error(e);
        } catch {
          // already closed or errored
        }
      });
    },
    pull(controller) {
      if (finished) return;
      if (pending) {
        controller.enqueue(pending);
        pending = null;
        return;
      }
      return new Promise<void>((resolve) => {
        waiting = resolve;
      });
    },
    cancel() {
      teardown();
    },
  }, { highWaterMark: 0 });
}

// ── capture ──

/**
 * A captured frame. `image` is a VideoFrame holding the camera's buffer: close() it when done (or
 * `using frame = ...`), or the camera may stall for want of buffers. Iterates as `[image, stamp]`.
 */
export class Frame {
  /** The picture. */
  readonly image: VideoFrameLike;
  /** Wall-clock capture time, s. */
  readonly stamp: number;
  /** Capture time on the camera's Clock, s. */
  readonly captured: number;
  /**
   * True when `captured` (and so ageS) is an estimate: the stream's timestamp offset rests on one
   * frame only (the first frame after an open or reopen reads as age ~0 even if it was delayed),
   * or the frame had no timestamp and its arrival time stands in. False once a second frame of the
   * same stream has confirmed the offset.
   */
  readonly ageEstimated: boolean;
  /** A url: camera's frame: the JPEG as the server sent it; null for other cameras. */
  readonly jpeg: Uint8Array | null;
  readonly #clock: Clock;
  readonly #canvas: CanvasFactory | undefined;

  constructor(image: VideoFrameLike, stamp: number, captured: number, clock: Clock, canvas?: CanvasFactory,
              ageEstimated = false, jpeg: Uint8Array | null = null) {
    this.image = image;
    this.stamp = stamp;
    this.captured = captured;
    this.#clock = clock;
    this.#canvas = canvas;
    this.ageEstimated = ageEstimated;
    this.jpeg = jpeg;
  }

  /** Seconds since the capture, on the monotonic clock (a wall-clock step cannot change it). */
  get ageS(): number {
    return Math.max(0, this.#clock.now() - this.captured);
  }

  get width(): number {
    return this.image.displayWidth;
  }

  get height(): number {
    return this.image.displayHeight;
  }

  /** JPEG bytes at `quality` (0-1), scaled to `size` (default: the frame's own size). Works in a page and a worker. */
  toJpeg(quality = 0.85, size?: { width: number; height: number } | null): Promise<Uint8Array> {
    return encodeJpeg(this.image, quality, size, this.#canvas);
  }

  /** Releases the picture's buffer. */
  close(): void {
    this.image.close();
  }

  [Symbol.dispose](): void {
    this.close();
  }

  *[Symbol.iterator](): Iterator<VideoFrameLike | number> {
    yield this.image;
    yield this.stamp;
  }
}

/** Camera settings and the environment it runs in (each injectable for tests). */
export interface CameraOptions {
  width?: number;
  height?: number;
  fps?: number;
  /** Default: navigator.mediaDevices. */
  mediaDevices?: MediaDevicesLike;
  /** Default: browserFrames. */
  frames?: FrameReader;
  /** Default: a RealClock. */
  clock?: Clock;
  /** Wall-clock seconds (default: Date.now() / 1000), for Frame.stamp only. */
  wallTime?: () => number;
  /** Default: an OffscreenCanvas, for Frame.toJpeg(). */
  canvas?: CanvasFactory;
  /** Default: console. */
  logger?: Logger;
  /** url: cameras: default globalThis.fetch. */
  fetch?: FetchLike;
  /** url: cameras: default decodeJpeg. */
  decode?: JpegDecoder;
}

/** Cameras open now, for the bandwidth hint when another fails to start. */
const liveCameras = new Set<Camera>();

/**
 * One camera captured in the background; the newest frame is kept, a dead camera reopened.
 *
 * Every open() and close() starts a new generation; work begun under an older one (a getUserMedia
 * still pending, a reopen timer, a frame in flight) gives its result up and touches nothing of
 * the current stream. A frame counts as new only when its capture timestamp advances: a stream
 * that repeats a picture's timestamp neither satisfies read() nor feeds the watchdog.
 */
export class Camera {
  readonly id: string | number;
  readonly width: number;
  readonly height: number;
  readonly fps: number;
  /** Why the camera is not delivering now, or null. */
  error: string | null = null;
  /** Times the camera was reopened after it ended or stopped sending. */
  reopens = 0;
  /** The capture mode the camera streams at (index into captureModes()). */
  mode = 0;

  readonly #md: MediaDevicesLike | undefined;
  readonly #frames: FrameReader;
  readonly #clock: Clock;
  readonly #wall: () => number;
  readonly #canvas: CanvasFactory | undefined;
  readonly #log: Logger;
  readonly #fetch: FetchLike | undefined;
  readonly #decode: JpegDecoder;
  readonly #signal = new Signal();
  #frame: VideoFrameLike | null = null;
  #jpeg: Uint8Array | null = null;
  #stamp = 0;
  #captured = 0;
  #estimated = true;
  #count = 0;
  #open = false;
  /** Bumped by open() and close(). */
  #generation = 0;
  /** The open() in progress, shared by concurrent callers. */
  #opening: Promise<void> | null = null;
  /** Bumped on every (re)start and release: callbacks of an older stream do nothing. */
  #session = 0;
  #stream: MediaStreamLike | null = null;
  #reader: ReadableStreamDefaultReader<VideoFrameLike> | null = null;
  /** A url: camera's HTTP stream. */
  #capture: HttpCapture | null = null;
  /** JPEGs skipped by url: captures already released. */
  #skippedBefore = 0;
  /** A stream (or url: capture) is open. */
  #live = false;
  /** Clock time of the last frame whose capture time advanced. */
  #lastFrame = 0;
  /** The current stream's timestamp calibration: offset, frames seen, last raw timestamp (µs). */
  #offset = Infinity;
  #calibrated = 0;
  #lastTs = -Infinity;
  #cancelWatch: (() => void) | null = null;
  #cancelReopen: (() => void) | null = null;
  #detach: (() => void) | null = null;

  constructor(id: string | number, options: CameraOptions = {}) {
    parseCameraId(id);
    this.id = id;
    this.width = options.width ?? 640;
    this.height = options.height ?? 480;
    this.fps = options.fps ?? 30;
    this.#md = options.mediaDevices;
    this.#frames = options.frames ?? browserFrames;
    this.#clock = options.clock ?? new RealClock();
    this.#wall = options.wallTime ?? (() => Date.now() / 1000);
    this.#canvas = options.canvas;
    this.#log = options.logger ?? console;
    this.#fetch = options.fetch;
    this.#decode = options.decode ?? decodeJpeg;
  }

  get isOpen(): boolean {
    return this.#open;
  }

  /** Frames captured since this camera was created (every one, whether or not anyone read it). */
  get frames(): number {
    return this.#count;
  }

  /**
   * For a url: camera, JPEGs that arrived but were replaced by a newer one before they were decoded
   * (only the newest is kept); 0 for other cameras.
   */
  get skipped(): number {
    return (this.#capture?.dropped ?? 0) + this.#skippedBefore;
  }

  /** The live capture stream (for a preview <video>), or null between streams. */
  get stream(): MediaStreamLike | null {
    return this.#stream;
  }

  /**
   * Open the device and start capturing; resolves once the first frame arrived. Concurrent calls
   * share one open. A close() before it completes makes it reject with StateError.
   */
  open(timeout = 5.0): Promise<void> {
    if (this.#opening) return this.#opening;
    if (this.#open) return Promise.resolve();
    const opening = this.#doOpen(timeout);
    this.#opening = opening;
    const clear = () => {
      if (this.#opening === opening) this.#opening = null;
    };
    opening.then(clear, clear);
    return opening;
  }

  async #doOpen(timeout: number): Promise<void> {
    const generation = ++this.#generation;
    this.#open = true;
    this.mode = 0;
    try {
      await this.#start(generation);
    } catch (e) {
      if (generation === this.#generation) {
        this.#open = false;
        this.#release();
      }
      throw e;
    }
    liveCameras.add(this);
    this.#watch(generation);
    let frame: Frame;
    try {
      frame = await this.read(timeout);
    } catch (e) {
      if (generation !== this.#generation) throw this.#closedWhileOpening();
      await this.close();
      if (e instanceof TimeoutError) throw new TimeoutError(`camera ${this.#name()} opened but sent no frame within ${timeout} s`);
      throw e;
    }
    frame.close();
    if (generation !== this.#generation) throw this.#closedWhileOpening();
  }

  #closedWhileOpening(): StateError {
    return new StateError(`camera ${this.#name()} was closed while opening`);
  }

  #name(): string {
    return typeof this.id === "number" ? `index:${this.id}` : this.id;
  }

  /**
   * getUserMedia at the current mode, falling back to smaller ones; then the frame reader. Touches
   * the camera's state only if `generation` is still current once the stream is there.
   */
  async #start(generation: number): Promise<void> {
    const url = urlOf(this.id);
    if (url !== null) return this.#startUrl(url, generation);
    const md = this.#md ?? navigatorMediaDevices();
    const deviceId = await resolveCamera(this.id, md);
    if (generation !== this.#generation) throw this.#closedWhileOpening();
    const modes = captureModes(this.width, this.height, this.fps);
    let stream: MediaStreamLike | null = null;
    let lastError: unknown = null;
    let mode = Math.min(this.mode, modes.length - 1);
    for (; mode < modes.length; mode++) {
      try {
        stream = await md.getUserMedia(cameraConstraints(deviceId, modes[mode]!));
        break;
      } catch (e) {
        lastError = e;
        if (generation !== this.#generation || FATAL_START.has(errorName(e))) break;
      }
    }
    const stopAll = () => {
      for (const t of stream?.getTracks() ?? []) t.stop();
    };
    if (generation !== this.#generation || !this.#open) {
      stopAll();
      throw this.#closedWhileOpening();
    }
    if (!stream) {
      const otherLive = [...liveCameras].some((c) => c !== this);
      throw new ConnectError(cameraStartError(this.#name(), lastError, otherLive), { cause: lastError });
    }
    const track = stream.getVideoTracks()[0];
    if (!track) {
      stopAll();
      throw new ConnectError(`cannot open camera ${this.#name()}: the stream has no video track`);
    }
    let readable: ReadableStream<VideoFrameLike>;
    try {
      readable = this.#frames(track);
    } catch (e) {
      stopAll();
      throw e;
    }
    this.mode = mode;
    const session = ++this.#session;
    this.#stream = stream;
    this.#live = true;
    this.#offset = Infinity;
    this.#calibrated = 0;
    this.#lastTs = -Infinity;
    this.#lastFrame = this.#clock.now();
    const onEnded = () => {
      if (session === this.#session) this.#dead(`camera ${this.#name()} ended (unplugged, or stopped by the system)`);
    };
    const onMute = () => {
      if (session === this.#session) this.error = `camera ${this.#name()} is muted (no frames from the system)`;
    };
    const onUnmute = () => {
      if (session === this.#session && this.error?.endsWith("(no frames from the system)")) this.error = null;
    };
    track.addEventListener("ended", onEnded);
    track.addEventListener("mute", onMute);
    track.addEventListener("unmute", onUnmute);
    this.#detach = () => {
      track.removeEventListener("ended", onEnded);
      track.removeEventListener("mute", onMute);
      track.removeEventListener("unmute", onUnmute);
    };
    const reader = readable.getReader();
    this.#reader = reader;
    void this.#pump(session, reader);
  }

  /** A url: camera: connect, then decode each newest JPEG the capture takes. */
  async #startUrl(url: string, generation: number): Promise<void> {
    const capture = await HttpCapture.open(url, { fps: this.fps, fetch: this.#fetch, clock: this.#clock, wallTime: this.#wall });
    if (generation !== this.#generation || !this.#open) {
      capture.release();
      throw this.#closedWhileOpening();
    }
    const session = ++this.#session;
    this.#capture = capture;
    this.#live = true;
    this.#lastFrame = this.#clock.now();
    void this.#pumpUrl(session, capture);
  }

  /**
   * Takes the capture's newest JPEG and decodes it. A capture that stopped leaves its error here;
   * the watchdog reopens it once READ_FAILURE_S passed without a frame.
   */
  async #pumpUrl(session: number, capture: HttpCapture): Promise<void> {
    while (session === this.#session) {
      const jpeg = await capture.read();
      if (session !== this.#session) return;
      if (jpeg === null) {
        if (capture.alive) continue;
        this.error = capture.error ?? this.error;
        return;
      }
      const [stamp, captured] = capture.lastTimes!;
      let image: VideoFrameLike | null = null;
      try {
        image = await this.#decode(jpeg, Math.round(captured * 1e6));
      } catch {
        // not a picture: no frame
      }
      if (session !== this.#session) {
        image?.close();
        return;
      }
      if (image) this.#store(image, Math.min(this.#clock.now(), captured), false, stamp, jpeg);
    }
  }

  async #pump(session: number, reader: ReadableStreamDefaultReader<VideoFrameLike>): Promise<void> {
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        if (session !== this.#session) {
          value.close();
          break;
        }
        this.#accept(value);
      }
    } catch (e) {
      if (session === this.#session) this.error = errorText(e);
    }
    if (session === this.#session && this.#open) this.#dead(this.error ?? `camera ${this.#name()} stopped returning frames`);
  }

  #accept(image: VideoFrameLike): void {
    const now = this.#clock.now();
    const ts = image.timestamp;
    let captured = now;
    let estimated = true;
    if (typeof ts === "number" && Number.isFinite(ts)) {
      if (ts <= this.#lastTs) {
        image.close(); // the same (or an older) capture again: not a new frame
        return;
      }
      this.#lastTs = ts;
      this.#offset = Math.min(this.#offset, now - ts / 1e6);
      this.#calibrated += 1;
      captured = Math.min(now, ts / 1e6 + this.#offset);
      estimated = this.#calibrated < 2;
    }
    this.#store(image, captured, estimated, this.#wall() - (now - captured), null);
  }

  /** The newest frame is `image`. */
  #store(image: VideoFrameLike, captured: number, estimated: boolean, stamp: number, jpeg: Uint8Array | null): void {
    const previous = this.#frame;
    this.#frame = image;
    this.#jpeg = jpeg;
    this.#captured = captured;
    this.#estimated = estimated;
    this.#stamp = stamp;
    this.#count += 1;
    this.#lastFrame = this.#clock.now();
    this.error = null;
    previous?.close();
    this.#signal.notify();
  }

  /** Checks every WATCH_S that new frames still arrive. */
  #watch(generation: number): void {
    this.#cancelWatch?.();
    const tick = () => {
      if (!this.#open || generation !== this.#generation) return;
      if (this.#live && this.#clock.now() - this.#lastFrame > READ_FAILURE_S) {
        this.#dead(this.error ?? `camera ${this.#name()} stopped returning frames`);
      }
      this.#cancelWatch = this.#clock.timer(WATCH_S, tick);
    };
    this.#cancelWatch = this.#clock.timer(WATCH_S, tick);
  }

  /** The stream is dead: drop its newest frame, release it and reopen after REOPEN_S. */
  #dead(reason: string): void {
    if (!this.#open || !this.#live) return;
    this.error = reason;
    this.#log.warn(`${reason}; reopening`);
    this.#release();
    this.#dropFrame();
    this.#scheduleReopen(this.#generation);
  }

  #dropFrame(): void {
    this.#frame?.close();
    this.#frame = null;
    this.#jpeg = null;
  }

  #scheduleReopen(generation: number): void {
    if (this.#cancelReopen) return;
    this.#cancelReopen = this.#clock.timer(REOPEN_S, () => {
      this.#cancelReopen = null;
      if (!this.#open || generation !== this.#generation) return;
      this.#start(generation).then(() => {
        if (generation !== this.#generation) return;
        this.reopens += 1;
        this.#log.info(`camera ${this.#name()} reopened`);
      }, (e: unknown) => {
        // A failed start assigned nothing; a stale one must not touch the current stream.
        if (!this.#open || generation !== this.#generation) return;
        this.error = errorText(e);
        this.#scheduleReopen(generation);
      });
    });
  }

  #release(): void {
    this.#session += 1;
    this.#live = false;
    const capture = this.#capture;
    this.#capture = null;
    if (capture) {
      capture.release();
      this.#skippedBefore += capture.dropped;
    }
    this.#detach?.();
    this.#detach = null;
    const reader = this.#reader;
    this.#reader = null;
    reader?.cancel().catch(() => undefined);
    const stream = this.#stream;
    this.#stream = null;
    for (const t of stream?.getTracks() ?? []) {
      try {
        t.stop();
      } catch {
        // already stopped
      }
    }
  }

  #current(): Frame {
    return new Frame(this.#frame!.clone(), this.#stamp, this.#captured, this.#clock, this.#canvas, this.#estimated, this.#jpeg);
  }

  /**
   * The next new frame (its capture time later than any before it); TimeoutError if none comes
   * within `timeout` s, StateError once closed. Close the frame when done.
   */
  async read(timeout = 1.0): Promise<Frame> {
    const seen = this.#count;
    const ok = await this.#signal.wait(() => this.#count > seen || !this.#open, timeout, this.#clock);
    if (!ok) throw new TimeoutError(`no frame from camera ${this.#name()} within ${timeout} s`);
    if (this.#count === seen || this.#frame === null || !this.#open) throw new StateError(`camera ${this.#name()} is closed`);
    return this.#current();
  }

  /**
   * The newest frame of the live stream as read() gives it (no wait), or null when there is none
   * (none yet, or the stream died) or it is older than `maxAgeS` (judged on the monotonic clock).
   * Close the frame when done.
   */
  newest(maxAgeS?: number | null): Frame | null {
    if (this.#frame === null) return null;
    if (maxAgeS !== undefined && maxAgeS !== null && this.#clock.now() - this.#captured > maxAgeS) return null;
    return this.#current();
  }

  /** The newest picture (no wait), or null when there is none or it is older than `maxAgeS`. Close it when done. */
  latest(maxAgeS?: number | null): VideoFrameLike | null {
    return this.newest(maxAgeS)?.image ?? null;
  }

  /**
   * The current picture as JPEG bytes: the newest frame, or the next one if that is older than
   * `maxAgeS`; TimeoutError when the next one is older than `maxAgeS` too.
   */
  async snapshot(maxAgeS = 0.5, quality = 0.85, size?: { width: number; height: number } | null): Promise<Uint8Array> {
    const frame = this.newest(maxAgeS) ?? (await this.read());
    try {
      if (frame.ageS > maxAgeS) {
        throw new TimeoutError(`camera ${this.#name()}: the newest frame is ${frame.ageS.toFixed(2)} s old (limit ${maxAgeS} s)`);
      }
      return await frame.toJpeg(quality, size);
    } finally {
      frame.close();
    }
  }

  /** Stop capturing and release the device and the newest frame. A read() or open() waiting rejects with StateError. */
  async close(): Promise<void> {
    this.#open = false;
    this.#generation += 1;
    this.#opening = null;
    liveCameras.delete(this);
    this.#cancelWatch?.();
    this.#cancelWatch = null;
    this.#cancelReopen?.();
    this.#cancelReopen = null;
    this.#release();
    this.#dropFrame();
    this.#signal.notify();
  }

  async [Symbol.asyncDispose](): Promise<void> {
    await this.close();
  }
}
