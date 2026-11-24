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
  /** Python: /dev/videoN; a browser has no device path. */
  path: null;
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
  | { kind: "usb" | "path"; path: string };

/** Parses an id (`device:`, `label:`, `index:N`, a number, or the Python SDK's `usb:` / `path:`). */
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
  throw new ValueError(`unknown camera id '${id}': expected device:<deviceId>, label:<text>, index:N, usb:<by-path> or path:<by-path>`);
}

async function videoInputs(md: MediaDevicesLike): Promise<MediaDeviceInfoLike[]> {
  return (await md.enumerateDevices()).filter((d) => d.kind === "videoinput");
}

/**
 * The deviceId an id names among the present cameras. A label or index needs the page's camera
 * permission: when labels are still hidden, one getUserMedia({video: true}) asks for it (its stream
 * is stopped at once) and the cameras are listed again.
 */
export async function resolveCamera(id: string | number, mediaDevices: MediaDevicesLike = navigatorMediaDevices()): Promise<string> {
  const parsed = parseCameraId(id);
  const shown = typeof id === "number" ? `index:${id}` : id;
  if (parsed.kind === "usb" || parsed.kind === "path") {
    throw new ConnectError(`camera ${shown} names a USB port, which a browser cannot see: name it device:<deviceId> or ` +
      "label:<text> (listCameras() lists both)");
  }
  if (parsed.kind === "device") return parsed.deviceId;
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
  readonly #clock: Clock;
  readonly #canvas: CanvasFactory | undefined;

  constructor(image: VideoFrameLike, stamp: number, captured: number, clock: Clock, canvas?: CanvasFactory,
              ageEstimated = false) {
    this.image = image;
    this.stamp = stamp;
    this.captured = captured;
    this.#clock = clock;
    this.#canvas = canvas;
    this.ageEstimated = ageEstimated;
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
  readonly #signal = new Signal();
  #frame: VideoFrameLike | null = null;
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
  }

  get isOpen(): boolean {
    return this.#open;
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
    const previous = this.#frame;
    this.#frame = image;
    this.#captured = captured;
    this.#estimated = estimated;
    this.#stamp = this.#wall() - (now - captured);
    this.#count += 1;
    this.#lastFrame = now;
    this.error = null;
    previous?.close();
    this.#signal.notify();
  }

  /** Checks every WATCH_S that new frames still arrive. */
  #watch(generation: number): void {
    this.#cancelWatch?.();
    const tick = () => {
      if (!this.#open || generation !== this.#generation) return;
      if (this.#stream && this.#clock.now() - this.#lastFrame > READ_FAILURE_S) {
        this.#dead(this.error ?? `camera ${this.#name()} stopped returning frames`);
      }
      this.#cancelWatch = this.#clock.timer(WATCH_S, tick);
    };
    this.#cancelWatch = this.#clock.timer(WATCH_S, tick);
  }

  /** The stream is dead: drop its newest frame, release it and reopen after REOPEN_S. */
  #dead(reason: string): void {
    if (!this.#open || !this.#stream) return;
    this.error = reason;
    this.#log.warn(`${reason}; reopening`);
    this.#release();
    this.#dropFrame();
    this.#scheduleReopen(this.#generation);
  }

  #dropFrame(): void {
    this.#frame?.close();
    this.#frame = null;
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
    return new Frame(this.#frame!.clone(), this.#stamp, this.#captured, this.#clock, this.#canvas, this.#estimated);
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
