// Port of the applicable parts of the Python SDK's tests/unit/test_cameras.py, on a fake
// navigator.mediaDevices, fake tracks and fake VideoFrames (no device is opened). Discovery tests
// of Linux sysfs and macOS system_profiler have no browser counterpart; enumerateDevices() stands
// in for them.
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ConnectError, StateError, TimeoutError, ValueError } from "../src/errors.js";
import { VirtualClock } from "../src/testing.js";
import {
  browserFrames, Camera, type CameraOptions, cameraConstraints, captureModes, type CanvasFactory, encodeJpeg, Frame,
  type FrameReader, listCameras, type MediaDeviceInfoLike, type MediaDevicesLike, type MediaStreamLike,
  type MediaStreamTrackLike, parseCameraId, READ_FAILURE_S, REOPEN_S, resolveCamera, type VideoFrameLike,
} from "../src/cameras.js";

class FakeVideoFrame implements VideoFrameLike {
  static live = 0;
  closed = false;
  constructor(readonly seq: number, readonly timestamp: number | null, readonly displayWidth = 640, readonly displayHeight = 480) {
    FakeVideoFrame.live++;
  }
  clone(): FakeVideoFrame {
    if (this.closed) throw new Error("InvalidStateError: clone of a closed frame");
    return new FakeVideoFrame(this.seq, this.timestamp, this.displayWidth, this.displayHeight);
  }
  close(): void {
    if (!this.closed) FakeVideoFrame.live--;
    this.closed = true;
  }
}

class FakeTrack implements MediaStreamTrackLike {
  readyState: "live" | "ended" = "live";
  muted = false;
  stopped = false;
  #listeners = new Map<string, Set<() => void>>();
  constructor(readonly constraints: MediaStreamConstraints) {}
  stop(): void {
    this.stopped = true;
    this.readyState = "ended"; // stop() fires no "ended" event, as in a browser
  }
  addEventListener(type: string, listener: () => void): void {
    if (!this.#listeners.has(type)) this.#listeners.set(type, new Set());
    this.#listeners.get(type)!.add(listener);
  }
  removeEventListener(type: string, listener: () => void): void {
    this.#listeners.get(type)?.delete(listener);
  }
  fire(type: string): void {
    for (const l of [...(this.#listeners.get(type) ?? [])]) l();
  }
}

class FakeStream implements MediaStreamLike {
  readonly track: FakeTrack;
  constructor(constraints: MediaStreamConstraints) {
    this.track = new FakeTrack(constraints);
  }
  getVideoTracks(): FakeTrack[] {
    return [this.track];
  }
  getTracks(): FakeTrack[] {
    return [this.track];
  }
}

const domError = (name: string, message = "") => Object.assign(new Error(message), { name });

const DEVICES: MediaDeviceInfoLike[] = [
  { deviceId: "mic", groupId: "g0", kind: "audioinput", label: "Mic" },
  { deviceId: "dev-wrist-l", groupId: "g1", kind: "videoinput", label: "Wrist Cam (0c45:6366)" },
  { deviceId: "dev-top", groupId: "g2", kind: "videoinput", label: "Top Cam: UVC (1bcf:2284)" },
  { deviceId: "dev-wrist-r", groupId: "g3", kind: "videoinput", label: "Wrist Cam (0c45:6366)" },
];

/** A fake camera system: devices, getUserMedia, and frame streams on the test's clock. */
class FakeCameras {
  devices: MediaDeviceInfoLike[] = DEVICES.map((d) => ({ ...d }));
  /** Labels and ids are hidden until the first getUserMedia (no permission yet). */
  permission = true;
  streams: FakeStream[] = [];
  constraints: MediaStreamConstraints[] = [];
  /** getUserMedia errors to throw, in order (null: succeed). */
  failures: Array<Error | null> = [];
  failAlways: Error | null = null;
  /** getUserMedia waits for this before answering (its outcome is decided when it is called). */
  gate: Promise<void> | null = null;
  /** From this frame on, every frame repeats this frame's capture timestamp (null: never). */
  repeatTsFrom: number | null = null;
  /** Frames a new stream sends before it freezes (null: never freezes). */
  freezeAfter: number | null = null;
  /** The browser clock's timestamps run this far ahead of the camera's Clock, s. */
  tsBase = 5000;
  /**
   * Delivery delay of frame n (s): its timestamp is that much before it arrives. Grow it by less
   * than a frame period per frame (ramp()), so capture times still advance as a camera's do.
   */
  delay: (n: number) => number = () => 0.01;
  frozen = new Set<FakeStream>();
  readers = 0;
  #controllers = new Map<FakeTrack, ReadableStreamDefaultController<VideoFrameLike>>();

  constructor(readonly clock: VirtualClock) {}

  get mediaDevices(): MediaDevicesLike {
    return {
      enumerateDevices: async () => this.devices.map((d) => (this.permission ? d : { ...d, deviceId: "", label: "" })),
      getUserMedia: async (constraints) => {
        this.constraints.push(constraints);
        const failure = this.failAlways ?? this.failures.shift() ?? null;
        if (this.gate) await this.gate;
        if (failure) throw failure;
        this.permission = true;
        const stream = new FakeStream(constraints);
        this.streams.push(stream);
        return stream;
      },
    };
  }

  frames: FrameReader = (track) => {
    this.readers++;
    const stream = this.streams.find((s) => s.track === track)!;
    const fakeTrack = track as FakeTrack;
    let seq = 0;
    let repeated: number | null = null;
    const repeatFrom = this.repeatTsFrom;
    let cancelled = false;
    // Pull-based (a frame is made only for a pending read), so no frame is left queued on cancel.
    return new ReadableStream<VideoFrameLike>({
      start: (controller) => {
        this.#controllers.set(fakeTrack, controller);
      },
      pull: async (controller) => {
        for (;;) {
          await this.clock.sleep(1 / 30);
          if (cancelled || fakeTrack.stopped || fakeTrack.readyState === "ended") return;
          if (!this.frozen.has(stream) && (this.freezeAfter === null || seq < this.freezeAfter)) {
            let ts = (this.clock.now() + this.tsBase - this.delay(seq)) * 1e6;
            if (repeatFrom !== null && seq >= repeatFrom) ts = repeated ??= ts;
            controller.enqueue(new FakeVideoFrame(seq++, ts));
            return;
          }
        }
      },
      cancel: () => {
        cancelled = true;
      },
    }, { highWaterMark: 0 });
  };

  /** The newest stream stops sending frames (track still live). */
  freeze(): void {
    this.frozen.add(this.streams.at(-1)!);
  }

  /** The newest stream's camera is unplugged. */
  unplug(): void {
    const track = this.streams.at(-1)!.track;
    track.readyState = "ended";
    track.fire("ended");
    try {
      this.#controllers.get(track)?.close();
    } catch {
      // already closed
    }
  }
}

/** A delivery delay of 0.01 s until frame `from`, then growing 0.03 s per frame up to `to` s. */
const ramp = (from: number, to: number) => (n: number) => (n < from ? 0.01 : Math.min(to, 0.01 + (n - from + 1) * 0.03));

function setup(options: Partial<CameraOptions> & { auto?: boolean } = {}) {
  const clock = new VirtualClock({ start: 100, auto: options.auto ?? true });
  const fake = new FakeCameras(clock);
  const logs: string[] = [];
  const logger = { info: (m: string) => logs.push(`info ${m}`), warn: (m: string) => logs.push(`warn ${m}`), error: (m: string) => logs.push(`error ${m}`) };
  let wall = 1_700_000_000;
  const make = (id: string | number = "device:dev-top", extra: Partial<CameraOptions> = {}) => new Camera(id, {
    mediaDevices: fake.mediaDevices, frames: fake.frames, clock, logger, wallTime: () => wall + clock.now(), ...options, ...extra,
  });
  return { clock, fake, logs, make, stepWall: (s: number) => (wall += s) };
}

beforeEach(() => {
  FakeVideoFrame.live = 0;
});

afterEach(() => {
  vi.unstubAllGlobals();
});

// ── discovery (enumerateDevices; nothing is opened) ──
describe("camera discovery", () => {
  it("lists video inputs by device id without opening them", async () => {
    const { fake } = setup();
    const found = await listCameras(fake.mediaDevices);
    expect(found.map((c) => [c.id, c.index, c.name])).toEqual([
      ["device:dev-wrist-l", 0, "Wrist Cam (0c45:6366)"],
      ["device:dev-top", 1, "Top Cam: UVC (1bcf:2284)"],
      ["device:dev-wrist-r", 2, "Wrist Cam (0c45:6366)"],
    ]);
    expect(found[0]!.path).toBeNull();
    expect(found[0]!.groupId).toBe("g1");
    expect(fake.constraints).toEqual([]);
  });

  it("before camera permission, cameras are listed by index", async () => {
    const { fake } = setup();
    fake.permission = false;
    expect((await listCameras(fake.mediaDevices)).map((c) => [c.id, c.name])).toEqual([
      ["index:0", "camera 0"], ["index:1", "camera 1"], ["index:2", "camera 2"],
    ]);
    expect(fake.constraints).toEqual([]);
  });

  it("parses every id form, the Python ones included", () => {
    expect(parseCameraId("index:4")).toEqual({ kind: "index", index: 4 });
    expect(parseCameraId(2)).toEqual({ kind: "index", index: 2 });
    expect(parseCameraId("device:abc:def")).toEqual({ kind: "device", deviceId: "abc:def" });
    expect(parseCameraId("label:Top Cam")).toEqual({ kind: "label", label: "Top Cam" });
    expect(parseCameraId("usb:platform-3610000.usb-usb-0:2.2.1:1.0")).toEqual({ kind: "usb", path: "platform-3610000.usb-usb-0:2.2.1:1.0" });
    expect(parseCameraId("path:platform-x")).toEqual({ kind: "path", path: "platform-x" });
    expect(() => parseCameraId("webcam")).toThrow(ValueError);
    expect(() => parseCameraId("index:x")).toThrow(ValueError);
    expect(() => new Camera("webcam")).toThrow(ValueError);
  });

  it("resolves ids to what getUserMedia opens", async () => {
    const { fake } = setup();
    const md = fake.mediaDevices;
    expect(await resolveCamera("index:1", md)).toBe("dev-top");
    expect(await resolveCamera(2, md)).toBe("dev-wrist-r");
    expect(await resolveCamera("device:anything", md)).toBe("anything");
    expect(await resolveCamera("label:Top Cam: UVC (1bcf:2284)", md)).toBe("dev-top");
    expect(await resolveCamera("label:top cam", md)).toBe("dev-top"); // a unique part of the label
    await expect(resolveCamera("label:Wrist Cam", md)).rejects.toThrow(/several cameras match label:Wrist Cam/);
    await expect(resolveCamera("label:Side", md)).rejects.toThrow(ConnectError);
    await expect(resolveCamera("index:9", md)).rejects.toThrow(/not connected/);
    await expect(resolveCamera("usb:platform-3610000.usb-usb-0:2.2.1:1.0", md)).rejects.toThrow(/name it device:<deviceId> or label:<text>/);
    expect(fake.constraints).toEqual([]);
  });

  it("a label needs camera permission: one stream asks for it and is stopped at once", async () => {
    const { fake } = setup();
    fake.permission = false;
    expect(await resolveCamera("label:Top", fake.mediaDevices)).toBe("dev-top");
    expect(fake.constraints).toEqual([{ video: true, audio: false }]);
    expect(fake.streams[0]!.track.stopped).toBe(true);
  });
});

describe("capture constraints", () => {
  it("asks for 640x480 @ 30 fps with ideal / max only, and falls back to smaller modes", () => {
    const modes = captureModes(640, 480, 30);
    expect(modes.map((m) => [m.width, m.height, m.frameRate])).toEqual([[640, 480, 30], [320, 240, 15], [320, 240, 10]]);
    expect(cameraConstraints("dev", modes[0]!)).toEqual({
      audio: false,
      video: { deviceId: { exact: "dev" }, width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30, max: 30 } },
    });
    expect(cameraConstraints(null, modes[1]!).video).not.toHaveProperty("deviceId");
    expect(captureModes(320, 240, 10).length).toBe(1);
  });
});

// ── capture (a fake mediaDevices and frame stream) ──
describe("camera capture", () => {
  it("open asks for the format before reading", async () => {
    const { fake, make, clock } = setup();
    const cam = make("index:1");
    await cam.open();
    try {
      expect(fake.constraints).toEqual([cameraConstraints("dev-top", captureModes(640, 480, 30)[0]!)]);
      const frame = await cam.read();
      const [image, stamp] = frame;
      expect(image).toBe(frame.image);
      expect(frame.width).toBe(640);
      expect(frame.height).toBe(480);
      expect(Math.abs((stamp as number) - (1_700_000_000 + clock.now()))).toBeLessThan(1.0);
      frame.close();
    } finally {
      await cam.close();
    }
    expect(fake.streams[0]!.track.stopped).toBe(true);
    expect(cam.isOpen).toBe(false);
    await clock.sleep(0.1); // a frame already handed over when the camera closed is closed when it arrives
    expect(FakeVideoFrame.live).toBe(0); // the camera released every frame it held
  });

  it("read waits for a new frame", async () => {
    const { make } = setup();
    await using cam = make();
    await cam.open();
    using first = await cam.read();
    using second = await cam.read();
    expect(second.stamp).toBeGreaterThanOrEqual(first.stamp);
    expect((second.image as FakeVideoFrame).seq).toBeGreaterThan((first.image as FakeVideoFrame).seq);
  });

  it("newest gives the frame and its stamp without waiting, and respects max age", async () => {
    const { make, fake } = setup();
    fake.delay = ramp(5, 2.01); // from frame 5 on, frames fall behind until they arrive 2 s after their capture
    const cam = make();
    expect(cam.newest()).toBeNull();
    await cam.open();
    try {
      const frame = await cam.read();
      const newest = cam.newest()!;
      expect(newest.stamp).toBeGreaterThanOrEqual(frame.stamp);
      expect(newest.image.displayWidth).toBe(frame.image.displayWidth);
      expect(cam.newest(1.0)).not.toBeNull();
      frame.close();
      newest.close();
      for (;;) {
        using f = await cam.read();
        if ((f.image as FakeVideoFrame).seq >= 80) break;
      }
      expect(cam.newest(1.0)).toBeNull();
      const old = cam.newest()!;
      expect(old.ageS).toBeGreaterThan(1.9);
      old.close();
      cam.latest()!.close();
    } finally {
      await cam.close();
    }
  });

  it("a wall-clock step does not make an old frame fresh", async () => {
    const { make, fake, stepWall } = setup();
    fake.delay = ramp(5, 10.01);
    const cam = make();
    await cam.open();
    try {
      const newest = cam.newest()!;
      const [image, stamp] = newest;
      expect(newest.image).toBe(image);
      expect(newest.stamp).toBe(stamp);
      expect(newest.ageS).toBeGreaterThanOrEqual(0);
      expect(newest.ageS).toBeLessThan(1.0);
      newest.close();
      stepWall(-3600); // the wall clock stepped back an hour
      for (;;) {
        using f = await cam.read();
        if ((f.image as FakeVideoFrame).seq >= 350) break;
      }
      expect(cam.newest(1.0)).toBeNull();
      const old = cam.newest()!;
      expect(old.ageS).toBeGreaterThanOrEqual(10);
      old.close();
    } finally {
      await cam.close();
    }
  });

  it("a dead stream's last frame is dropped: newest() gives nothing stale", async () => {
    const { make, fake, clock } = setup();
    const cam = make();
    await cam.open();
    try {
      fake.failAlways = domError("NotFoundError");
      fake.unplug();
      expect(cam.newest()).toBeNull();
      expect(cam.newest(100)).toBeNull();
      expect(cam.latest()).toBeNull();
      fake.failAlways = null;
      await clock.sleep(REOPEN_S * 2 + 0.5);
      using fresh = cam.newest();
      expect(fresh).not.toBeNull();
    } finally {
      await cam.close();
    }
    const frozen = make();
    await frozen.open();
    try {
      fake.failAlways = domError("NotReadableError");
      fake.freeze();
      await clock.sleep(READ_FAILURE_S + 0.3);
      expect(frozen.newest()).toBeNull();
    } finally {
      await frozen.close();
    }
  });

  it("frames whose capture time does not advance are not new and do not feed the watchdog", async () => {
    const { make, fake, clock, logs } = setup();
    fake.repeatTsFrom = 3;
    const cam = make();
    await cam.open();
    try {
      for (;;) {
        using f = await cam.read();
        if ((f.image as FakeVideoFrame).seq >= 3) break;
      }
      fake.repeatTsFrom = null; // a reopened stream sends normally again
      await expect(cam.read(0.5)).rejects.toThrow(TimeoutError); // repeats keep arriving, none is new
      await clock.sleep(READ_FAILURE_S);
      expect(logs.some((l) => l.includes("stopped returning frames; reopening"))).toBe(true);
      await clock.sleep(REOPEN_S + 0.2);
      expect(cam.reopens).toBe(1);
      (await cam.read()).close();
    } finally {
      await cam.close();
    }
  });

  it("the first frame's age is an estimate until a second frame confirms the offset", async () => {
    const { make, fake } = setup();
    fake.delay = (n) => (n === 0 ? 0.5 : 0.01); // the first frame was delayed: its age reads ~0 anyway
    const cam = make();
    await cam.open();
    try {
      const first = cam.newest()!;
      expect((first.image as FakeVideoFrame).seq).toBe(0);
      expect(first.ageEstimated).toBe(true);
      expect(first.ageS).toBeLessThan(0.1);
      first.close();
      using second = await cam.read();
      expect(second.ageEstimated).toBe(false);
    } finally {
      await cam.close();
    }
  });

  it("snapshot refuses a fallback frame older than its limit", async () => {
    const { make, fake } = setup({ canvas: () => ({
      getContext: () => ({ drawImage: () => undefined }),
      convertToBlob: async () => ({ type: "image/jpeg", arrayBuffer: async () => new ArrayBuffer(1) }),
    }) });
    fake.delay = ramp(2, 3.01);
    const cam = make();
    await cam.open();
    try {
      for (;;) {
        using f = await cam.read();
        if ((f.image as FakeVideoFrame).seq >= 120) break;
      }
      await expect(cam.snapshot(0.5)).rejects.toThrow(/newest frame is 3\.\d+ s old \(limit 0.5 s\)/);
      expect((await cam.snapshot(5)).length).toBe(1);
    } finally {
      await cam.close();
    }
  });

  it("ages come from the capture time, not the delivery", async () => {
    const { make, fake } = setup();
    fake.delay = ramp(10, 0.21); // from frame 10 on, frames fall 0.2 s behind the first ones
    await using cam = make();
    await cam.open();
    for (;;) {
      using frame = await cam.read();
      if ((frame.image as FakeVideoFrame).seq < 20) continue;
      expect(frame.ageS).toBeGreaterThan(0.19);
      expect(frame.ageS).toBeLessThan(0.21);
      break;
    }
  });

  it("a camera that stops sending is reopened", async () => {
    const { make, fake, clock, logs } = setup();
    const cam = make();
    await cam.open();
    try {
      fake.freeze();
      await clock.sleep(READ_FAILURE_S + REOPEN_S + 1);
      expect(cam.reopens).toBeGreaterThanOrEqual(1);
      expect(fake.streams[0]!.track.stopped).toBe(true);
      expect(fake.streams.length).toBeGreaterThanOrEqual(2);
      expect(logs.some((l) => l.startsWith("warn") && l.includes("stopped returning frames; reopening"))).toBe(true);
      const frame = await cam.read();
      frame.close();
      expect(cam.error).toBeNull();
    } finally {
      await cam.close();
    }
  });

  it("an unplugged camera (ended track) is reopened after REOPEN_S, and keeps retrying while it is gone", async () => {
    const { make, fake, clock } = setup();
    const cam = make();
    await cam.open();
    try {
      fake.failures = [domError("NotFoundError", "Requested device not found"), domError("NotFoundError")];
      fake.unplug();
      expect(cam.error).toMatch(/ended/);
      await clock.sleep(REOPEN_S * 2 + 0.5);
      expect(cam.reopens).toBe(0);
      expect(cam.error).toMatch(/NotFoundError/);
      await clock.sleep(REOPEN_S + 0.5);
      expect(cam.reopens).toBe(1);
      (await cam.read()).close();
    } finally {
      await cam.close();
    }
  });

  it("a camera that cannot start at 640x480 falls back to a smaller mode and stays there", async () => {
    const { make, fake, clock } = setup();
    fake.failures = [domError("NotReadableError", "Could not start video source")];
    const cam = make();
    await cam.open();
    try {
      expect(cam.mode).toBe(1);
      expect((fake.constraints[1]!.video as MediaTrackConstraints).width).toEqual({ ideal: 320 });
      fake.freeze();
      await clock.sleep(READ_FAILURE_S + REOPEN_S + 0.5);
      expect(cam.reopens).toBe(1);
      expect((fake.constraints.at(-1)!.video as MediaTrackConstraints).width).toEqual({ ideal: 320 });
    } finally {
      await cam.close();
    }
  });

  it("a camera that will not open throws", async () => {
    const { make, fake } = setup();
    fake.failAlways = domError("NotReadableError", "Could not start video source");
    await expect(make().open()).rejects.toThrow(ConnectError);
    expect(fake.constraints.length).toBe(3); // every mode
    const error = (await make().open().then(() => null, (e: unknown) => e)) as Error;
    expect(error.message).toMatch(/^cannot open camera device:dev-top \(NotReadableError\)/);
    fake.failAlways = domError("NotAllowedError", "Permission denied");
    fake.constraints = [];
    await expect(make().open()).rejects.toThrow(/allow camera access/);
    expect(fake.constraints.length).toBe(1); // no smaller mode fixes a refusal
  });

  it("a camera with no frames fails open and releases", async () => {
    const { make, fake } = setup();
    fake.freezeAfter = 0;
    const cam = make();
    await expect(cam.open(0.1)).rejects.toThrow(TimeoutError);
    await expect(make().open(0.1)).rejects.toThrow(/opened but sent no frame within 0.1 s/);
    expect(cam.isOpen).toBe(false);
    expect(fake.streams[0]!.track.stopped).toBe(true);
  });

  it("read on a closed camera fails fast", async () => {
    const { make } = setup();
    const cam = make();
    await cam.open();
    await cam.close();
    await expect(cam.read(0.2)).rejects.toThrow(StateError);
    const pending = make();
    await pending.open();
    const waiting = pending.read(5);
    await pending.close();
    await expect(waiting).rejects.toThrow(StateError);
  });

  it("frames are clones the caller closes; the camera keeps only the newest", async () => {
    const { make, clock } = setup();
    const cam = make();
    await cam.open();
    const held: Frame[] = [];
    for (let i = 0; i < 5; i++) held.push(await cam.read());
    await cam.close();
    await clock.sleep(0.1);
    expect(FakeVideoFrame.live).toBe(5); // the clones; every frame the camera held is released
    expect(held.every((f) => !(f.image as FakeVideoFrame).closed)).toBe(true);
    for (const f of held) f.close();
    expect(FakeVideoFrame.live).toBe(0);
  });
});

describe("JPEG encoding", () => {
  function fakeCanvas() {
    const made: Array<{ width: number; height: number; drawn: unknown[]; options?: unknown }> = [];
    const canvas: CanvasFactory = (width, height) => {
      const record: { width: number; height: number; drawn: unknown[]; options?: unknown } = { width, height, drawn: [] };
      made.push(record);
      return {
        getContext: () => ({ drawImage: (...args: unknown[]) => void record.drawn.push(args) }),
        convertToBlob: async (options) => {
          record.options = options;
          return { type: "image/jpeg", arrayBuffer: async () => new Uint8Array([0xff, 0xd8, 0xff, 0xd9]).buffer };
        },
      };
    };
    return { made, canvas };
  }

  it("toJpeg draws the frame at the asked size and gives the bytes", async () => {
    const { made, canvas } = fakeCanvas();
    const { make } = setup({ canvas });
    await using cam = make();
    await cam.open();
    using frame = await cam.read();
    const bytes = await frame.toJpeg(0.85, { width: 320, height: 176 });
    expect([...bytes]).toEqual([0xff, 0xd8, 0xff, 0xd9]);
    expect(made[0]).toMatchObject({ width: 320, height: 176, options: { type: "image/jpeg", quality: 0.85 } });
    expect((made[0]!.drawn[0] as unknown[]).slice(1)).toEqual([0, 0, 320, 176]);
    await frame.toJpeg();
    expect(made[1]).toMatchObject({ width: 640, height: 480 });
  });

  it("snapshot encodes the newest frame and releases it", async () => {
    const { made, canvas } = fakeCanvas();
    const { make, clock } = setup({ canvas });
    const cam = make();
    await cam.open();
    const bytes = await cam.snapshot();
    expect(bytes.length).toBe(4);
    expect(made.length).toBe(1);
    await cam.close();
    await clock.sleep(0.1);
    expect(FakeVideoFrame.live).toBe(0); // the snapshot's frame was released
  });

  it("refuses a canvas that cannot encode JPEG", async () => {
    const canvas: CanvasFactory = () => ({
      getContext: () => ({ drawImage: () => undefined }),
      convertToBlob: async () => ({ type: "image/png", arrayBuffer: async () => new ArrayBuffer(0) }),
    });
    await expect(encodeJpeg(new FakeVideoFrame(0, 0), 0.8, null, canvas)).rejects.toThrow(/JPEG encoding unavailable/);
  });

  it("uses OffscreenCanvas by default", async () => {
    const sizes: number[][] = [];
    vi.stubGlobal("OffscreenCanvas", class {
      constructor(w: number, h: number) {
        sizes.push([w, h]);
      }
      getContext() {
        return { drawImage: () => undefined };
      }
      async convertToBlob() {
        return { type: "image/jpeg", arrayBuffer: async () => new ArrayBuffer(2) };
      }
    });
    expect((await encodeJpeg(new FakeVideoFrame(0, 0))).length).toBe(2);
    expect(sizes).toEqual([[640, 480]]);
  });
});


describe("camera open races", () => {
  const gate = () => {
    let open!: () => void;
    const promise = new Promise<void>((resolve) => (open = resolve));
    return { promise, open };
  };

  it("an open superseded by close() and a new open stops only its own stream", async () => {
    const { make, fake } = setup();
    const cam = make();
    const g = gate();
    fake.gate = g.promise;
    const a = cam.open();
    await Promise.resolve();
    await new Promise((r) => setImmediate(r));
    expect(fake.constraints.length).toBe(1); // A is waiting in getUserMedia
    await cam.close();
    fake.gate = null;
    const b = cam.open();
    await b; // B's stream is up and delivered a frame
    const bStream = cam.stream as FakeStream;
    g.open(); // A's getUserMedia answers now
    await expect(a).rejects.toThrow(/closed while opening/);
    expect(fake.streams.find((st) => st !== bStream)!.track.stopped).toBe(true); // A's own stream
    expect(cam.stream).toBe(bStream);
    expect(bStream.track.stopped).toBe(false);
    expect(cam.isOpen).toBe(true);
    (await cam.read()).close();
    await cam.close();
  });

  it("concurrent open() calls share one open and resolve after the first frame", async () => {
    const { make, fake } = setup();
    const cam = make();
    const first = cam.open();
    const second = cam.open();
    expect(second).toBe(first);
    await second;
    expect(fake.constraints.length).toBe(1);
    expect(cam.newest()).not.toBeNull();
    cam.newest()!.close();
    await cam.open(); // already open
    expect(fake.constraints.length).toBe(1);
    await cam.close();
  });

  it("a stale reopen that fails leaves the next session's stream alone", async () => {
    const { make, fake, clock } = setup();
    const cam = make();
    await cam.open();
    const g = gate();
    fake.gate = g.promise;
    fake.failures = [domError("NotReadableError")];
    fake.unplug();
    await clock.sleep(REOPEN_S + 0.1); // the reopen is waiting in getUserMedia, bound to fail
    const before = fake.constraints.length;
    expect(before).toBeGreaterThan(1);
    await cam.close();
    fake.gate = null;
    await cam.open();
    const bStream = cam.stream as FakeStream;
    g.open();
    await clock.sleep(REOPEN_S * 3);
    expect(cam.stream).toBe(bStream);
    expect(bStream.track.stopped).toBe(false);
    expect(cam.error).toBeNull();
    expect(cam.reopens).toBe(0);
    (await cam.read()).close();
    await cam.close();
  });
});

describe("the video-element frame reader", () => {
  class FakeVideo {
    muted = false;
    playsInline = false;
    srcObject: unknown = null;
    paused = false;
    callbacks = new Map<number, (now: number, metadata: { captureTime?: number }) => void>();
    cancelled: number[] = [];
    next = 1;
    playResult: Promise<void> = Promise.resolve();
    play() {
      return this.playResult;
    }
    pause() {
      this.paused = true;
    }
    requestVideoFrameCallback(cb: (now: number, metadata: { captureTime?: number }) => void) {
      const h = this.next++;
      this.callbacks.set(h, cb);
      return h;
    }
    cancelVideoFrameCallback(h: number) {
      this.cancelled.push(h);
      this.callbacks.delete(h);
    }
    /** The browser presents a frame captured at `at` ms. */
    present(at: number) {
      const [h, cb] = [...this.callbacks][0]!;
      this.callbacks.delete(h);
      cb(at + 5, { captureTime: at });
    }
  }

  function stub(playResult?: Promise<void>) {
    const video = new FakeVideo();
    if (playResult) video.playResult = playResult;
    const made: FakeVideoFrame[] = [];
    vi.stubGlobal("document", { createElement: () => video });
    vi.stubGlobal("MediaStream", class {
      constructor(readonly tracks: unknown[]) {}
    });
    vi.stubGlobal("VideoFrame", class extends FakeVideoFrame {
      constructor(_source: unknown, init: { timestamp: number }) {
        super(made.length, init.timestamp);
        made.push(this);
      }
    });
    return { video, made };
  }

  const listeners = (track: FakeTrack) => {
    let n = 0;
    const original = track.addEventListener.bind(track);
    const remove = track.removeEventListener.bind(track);
    track.addEventListener = (t: string, l: () => void) => {
      n++;
      original(t, l);
    };
    track.removeEventListener = (t: string, l: () => void) => {
      n--;
      remove(t, l);
    };
    return () => n;
  };

  it("delivers frames stamped with their capture time and keeps only the newest undelivered one", async () => {
    const { video, made } = stub();
    const reader = browserFrames(new FakeTrack({})).getReader();
    video.present(1000);
    video.present(1033); // nobody was reading: the first is replaced and closed
    expect(made[0]!.closed).toBe(true);
    const { value } = await reader.read();
    expect(value).toBe(made[1]);
    expect(value!.timestamp).toBe(1_033_000);
    const pending = reader.read();
    video.present(1066); // straight to the waiting read
    expect((await pending).value).toBe(made[2]);
    value!.close();
    made[2]!.close();
    await reader.cancel();
  });

  it("an ended track tears down once, removes its listener and closes the undelivered frame", async () => {
    const { video, made } = stub();
    const track = new FakeTrack({});
    const count = listeners(track);
    const reader = browserFrames(track).getReader();
    expect(count()).toBe(1);
    video.present(1000);
    track.fire("ended");
    expect(count()).toBe(0);
    expect(made[0]!.closed).toBe(true);
    expect(video.paused).toBe(true);
    expect(video.srcObject).toBeNull();
    expect(video.callbacks.size).toBe(0);
    expect((await reader.read()).done).toBe(true);
    const cancelled = video.cancelled.length;
    await reader.cancel(); // nothing more to undo
    expect(video.cancelled.length).toBe(cancelled);
  });

  it("a failed play() errors the stream and tears down", async () => {
    const { video } = stub(Promise.reject(Object.assign(new Error("play() failed"), { name: "NotAllowedError" })));
    const track = new FakeTrack({});
    const count = listeners(track);
    const reader = browserFrames(track).getReader();
    await expect(reader.read()).rejects.toThrow(/play\(\) failed/);
    expect(count()).toBe(0);
    expect(video.srcObject).toBeNull();
    expect(video.callbacks.size).toBe(0);
  });

  it("cancel closes a frame that was never delivered", async () => {
    const { video, made } = stub();
    const track = new FakeTrack({});
    const count = listeners(track);
    const reader = browserFrames(track).getReader();
    video.present(1000);
    await reader.cancel();
    expect(made[0]!.closed).toBe(true);
    expect(count()).toBe(0);
    expect(video.callbacks.size).toBe(0);
    expect(FakeVideoFrame.live).toBe(0);
  });
});

describe("browser frame readers", () => {
  it("prefers MediaStreamTrackProcessor", () => {
    const readable = new ReadableStream<VideoFrameLike>();
    const seen: unknown[] = [];
    vi.stubGlobal("MediaStreamTrackProcessor", class {
      readable = readable;
      constructor(init: { track: unknown }) {
        seen.push(init.track);
      }
    });
    const track = new FakeTrack({});
    expect(browserFrames(track)).toBe(readable);
    expect(seen).toEqual([track]);
  });

  it("without a processor or a page, says what is needed", () => {
    expect(() => browserFrames(new FakeTrack({}))).toThrow(/MediaStreamTrackProcessor/);
  });

  it("without navigator.mediaDevices, opening says where cameras work", async () => {
    const cam = new Camera("index:0", { clock: new VirtualClock() });
    await expect(cam.open()).rejects.toThrow(/navigator.mediaDevices/);
    expect(cam.isOpen).toBe(false);
  });
});
