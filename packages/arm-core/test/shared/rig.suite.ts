// Port of the Python SDK's tests/unit/test_rig.py: rig descriptions (round trips, validation, lookups)
// and ArmGroup on simulated arms.
import { readFileSync } from "node:fs";
import { afterEach, describe, expect, it, vi } from "vitest";
import { type Arm, ArmConfig, ArmError, type CanBackend, type Product } from "@makermods/arm-core";
import { FakeGsUsbDevice, VirtualClock } from "@makermods/arm-core/testing";
import { GsUsbBackend } from "../../src/transport/gs-usb.js";
import { SlcanSerialBackend } from "../../src/transport/slcan-serial.js";
import { ArmLeader, StarLeader } from "../../src/leaders/index.js";
import type { SerialPortLike } from "../../src/transport/serial.js";
import { ArmGroup, ArmSpec, LeaderSpec, Rig, RigError, type RigOptions } from "../../src/rig.js";
import { type ArmFakes, type FakeMotorLike, RecordingLogger } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function rigSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const { product, FakeBus, FakeMotor, makeArm, twoJointConfig } = fk;
  /** A Star mapping naming the product's joints. */
  const starMap = () => ({
    joints: fk.defaultConfig().joints.map((j, i) => ({ name: j.label, servo: i + 1, direction: 1, range_deg: [-90, 90] })),
  });

  function sample(options: RigOptions = {}): Rig {
    return new Rig({
      arms: {
        left: new ArmSpec("gs_usb", { port: "gs_usb:0036" }),
        right: new ArmSpec("socketcan", { channel: "can1" }),
        spare: new ArmSpec("slcan", { port: "/dev/ttyACM0", profile: "spare.yaml", baudrate: 921600 }),
      },
      cameras: { top: "usb:platform-1.2", left_wrist: "index:2" },
      leaders: {
        left: new LeaderSpec("star", { port: "/dev/ttyUSB0", map: "maps/star.json" }),
        right: new LeaderSpec("arm", { arm: "spare" }),
      },
    }, product, options);
  }

  /** Backends from FakeBus([]), recording what each entry asked for. */
  function fakeBackends(made: Array<[string, unknown]> = []): RigOptions["backends"] {
    return (spec) => {
      made.push([spec.backend, spec.backendKwargs()]);
      return new FakeBus([]) as CanBackend;
    };
  }

  const fakePort = (): SerialPortLike => ({
    open: async () => undefined, close: async () => undefined, readable: null, writable: null,
  });

  describe("rig", () => {
    it("round trip", () => {
      const raw = sample().toRaw();
      const loaded = Rig.fromRaw(JSON.stringify(raw), product, { path: "sub/rig.json" });
      expect(loaded.toRaw()).toEqual(raw);
      expect(loaded.armNames).toEqual(["left", "right", "spare"]);
      expect(loaded.path).toBe("sub/rig.json");
      expect(raw.arms.left).toEqual({ backend: "gs_usb", port: "gs_usb:0036" }); // unset keys are left out
      expect(raw.arms.spare).toEqual({ backend: "slcan", port: "/dev/ttyACM0", profile: "spare.yaml", baudrate: 921600 });
    });

    it("an empty description is an empty rig", () => {
      // Python: a missing rig file is an empty rig; here there is no file, only its contents.
      for (const raw of [null, undefined, {}, "{}"]) expect(Rig.fromRaw(raw, product).arms.size).toBe(0);
    });

    const cases: Array<[unknown, RegExp]> = [
      [{ arms: { a: { backend: "can" } } }, /backend must be one of/],
      [{ arms: { a: { backend: "gs_usb", speed: 1 } } }, /unknown keys \['speed'\]/],
      [{ arms: ["a", "b"] }, /expected a mapping/],
      [{ arms: { a: { port: "x" } } }, /missing 1 required key: 'backend'/],
      [{ robots: {} }, /expected a mapping with arms/],
      [{ arms: { a: { backend: "gs_usb" } }, leaders: { b: { type: "star", port: "/dev/x" } } }, /no arm named 'b' to follow/],
      [{ arms: { a: { backend: "gs_usb" } }, leaders: { a: { type: "arm", arm: "a" } } }, /cannot lead itself/],
      [{ arms: { a: { backend: "gs_usb" } }, leaders: { a: { type: "arm", arm: "z" } } }, /no arm named 'z'/],
      [{ arms: { a: { backend: "gs_usb" } }, leaders: { a: { type: "star" } } }, /needs a port/],
      [{ arms: { a: { backend: "gs_usb" } }, leaders: { a: { type: "drone" } } }, /leader type must be one of/],
      [{ arms: { a: { backend: "socketcan" }, b: { backend: "socketcan", channel: "can0" } } }, /same bus can0/],
      [{ arms: { all: { backend: "gs_usb" } } }, /cannot name an arm/],
      ['{"arms": {"a": {"backend": "gs_usb"}', /not valid JSON/],
      [[1, 2], /expected a mapping with arms/],
    ];
    for (const [raw, message] of cases) {
      it(`validation: ${message.source}`, () => {
        expect(() => Rig.fromRaw(raw, product)).toThrow(RigError);
        expect(() => Rig.fromRaw(raw, product)).toThrow(message);
      });
    }

    it("validate refuses an invalid rig", () => {
      // Python: save() validates first and writes nothing; there is no save() here.
      const rig = sample();
      rig.leaders.set("ghost", new LeaderSpec("star", { port: "/dev/x" }));
      expect(() => rig.validate()).toThrow(RigError);
    });

    it("names and lookups", () => {
      const rig = sample();
      expect(rig.names(["all"])).toEqual(["left", "right", "spare"]);
      expect(rig.names(["right", "left", "right"])).toEqual(["right", "left"]);
      expect(() => rig.names(["middle"])).toThrow(/arms: left, right, spare/);
      expect(() => rig.names([])).toThrow(/no arms in the rig/);
      expect(rig.cameraId("top")).toBe("usb:platform-1.2");
      expect(rig.cameraId("index:5")).toBe("index:5");
      expect(rig.cameraId("device:abc")).toBe("device:abc");
      expect(rig.cameraId("label:Top Cam")).toBe("label:Top Cam");
      expect(rig.cameraId("url:http://127.0.0.1:8090/front/stream.mjpg")).toBe("url:http://127.0.0.1:8090/front/stream.mjpg");
      expect(rig.cameraId("url:https://host/a.jpg")).toBe("url:https://host/a.jpg");
      expect(() => rig.cameraId("url:ftp://host/x")).toThrow(/no camera named/);
      expect(() => rig.cameraId("side")).toThrow(/cameras: top, left_wrist/);
      expect(rig.camera("top").id).toBe("usb:platform-1.2"); // not opened
      expect(rig.camera("top").isOpen).toBe(false);
      expect(rig.spec("right").address).toBe("can1");
      expect(new ArmSpec("gs_usb").address).toBe("gs_usb");
    });

    it("backend kwargs", () => {
      expect(new ArmSpec("socketcan").backendKwargs()).toEqual({ channel: "can0" });
      expect(new ArmSpec("gs_usb").backendKwargs()).toEqual({ port: null });
      const kw = new ArmSpec("slcan", { port: "/dev/ttyACM1", baudrate: 921600 }).backendKwargs() as Record<string, unknown>;
      expect(kw.port).toBe("/dev/ttyACM1");
      expect(kw.baudrate).toBe(921600);
      expect(kw.rtscts).toBe(false);
      expect(kw.startupDelay).toBe(2.5);
    });

    it("arm builds from the entry with a profile loaded by path", async () => {
      const made: Array<[string, unknown]> = [];
      const loaded: string[] = [];
      const files = (path: string) => {
        loaded.push(path);
        if (path !== "mine.yaml") throw new Error(`ENOENT: ${path}`);
        return product.defaultProfile;
      };
      const rig = new Rig({ arms: { left: new ArmSpec("gs_usb", { port: "gs_usb:X", profile: "mine.yaml" }) } }, product,
        { path: "rig.yaml", files, backends: fakeBackends(made) });
      const arm = await rig.arm("left");
      expect(made).toEqual([["gs_usb", { port: "gs_usb:X" }]]);
      expect(loaded).toEqual(["mine.yaml"]);
      expect(arm.state).toBe("disconnected");
      expect(arm.config.nJoints).toBe(fk.defaultConfig().nJoints);
      rig.arms.get("left")!.profile = "missing.yaml";
      await expect(rig.arm("left")).rejects.toThrow(/ENOENT/);
      rig.arms.get("left")!.profile = null; // the shipped profile
      expect((await rig.arm("left")).config.nJoints).toBe(fk.defaultConfig().nJoints);
      await expect(new Rig({ arms: { a: new ArmSpec("gs_usb", { profile: "x.yaml" }) } }, product,
        { backends: fakeBackends() }).arm("a")).rejects.toThrow(/pass RigOptions.files/);
    });

    it("the built-in resolver builds mock backends and refuses socketcan", async () => {
      const rig = Rig.fromRaw({ arms: { a: { backend: "mock" }, b: { backend: "socketcan" }, c: { backend: "slcan" } } }, product);
      expect((await rig.openBackend("a")).constructor.name).toBe("MockBackend");
      await expect(rig.openBackend("b")).rejects.toThrow(/socketcan/);
      await expect(rig.openBackend("c")).rejects.toThrow(/Web Serial is not available/); // default: SlcanSerialBackend on a granted port
      const ports: Array<[string | null, string]> = [];
      const built: unknown[] = [];
      const withSlcan = Rig.fromRaw({ arms: { c: { backend: "slcan", baudrate: 921600 } } }, product, {
        serialPorts: async (port, purpose) => {
          ports.push([port, purpose]);
          return fakePort();
        },
        slcan: (_port, options) => {
          built.push(options);
          return new FakeBus([]) as CanBackend;
        },
      });
      await withSlcan.openBackend("c");
      expect(ports).toEqual([[null, "slcan"]]);
      expect(built).toEqual([{ rtscts: false, baudrate: 921600, startupDelay: 2.5 }]);
    });

    it("leaders", async () => {
      const ports: Array<[string | null, string]> = [];
      const port = fakePort();
      const files = (path: string) => {
        if (path === "maps/star.json") return starMap();
        if (path === "spare.yaml") return product.defaultProfile;
        throw new Error(`ENOENT: ${path}`);
      };
      const rig = sample({
        path: "rig.yaml", backends: fakeBackends(), serialPorts: async (p, purpose) => {
          ports.push([p, purpose]);
          return port;
        },
      });
      const follower = await rig.arm("left");
      await expect(rig.leader("left", follower)).rejects.toThrow(/pass RigOptions.files/); // the map is loaded by path
      rig.options.files = files;
      const star = await rig.leader("left", follower);
      expect(star).toBeInstanceOf(StarLeader);
      expect((star as StarLeader).port).toBe(port);
      expect((star as StarLeader).bus).toBeNull();
      expect(ports).toEqual([["/dev/ttyUSB0", "star"]]);
      const spare = await rig.arm("spare");
      const leader = await rig.leader("right", await rig.arm("right"), new Map([["spare", spare]]));
      expect(leader).toBeInstanceOf(ArmLeader);
      expect((leader as ArmLeader).arm).toBe(spare);
      await expect(rig.leader("spare", spare)).rejects.toThrow(/no leader for arm 'spare'/);
    });

    it("the built-in serial resolver cannot find a port by path", async () => {
      const { browserSerialPorts } = await import("../../src/rig.js");
      const a = { ...fakePort(), getInfo: () => ({ usbVendorId: 0x1a86, usbProductId: 0x7523 }) };
      const b = { ...fakePort(), getInfo: () => ({ usbVendorId: 0x0483, usbProductId: 0x5740 }) };
      const resolve = browserSerialPorts(async () => [a, b]);
      await expect(resolve("/dev/ttyUSB0", "star")).rejects.toThrow(/cannot find serial port '\/dev\/ttyUSB0'/);
      expect(await resolve("serial:1a86:7523", "star")).toBe(a);
      await expect(resolve(null, "slcan")).rejects.toThrow(/several serial ports/);
      expect(await browserSerialPorts(async () => [b])(null, "slcan")).toBe(b);
      await expect(browserSerialPorts(async () => [])(null, "slcan")).rejects.toThrow(/no USB CAN adapter found/);
    });


    // ── review fixes: one config per arm, one physical bus per owner ──
    afterEach(() => {
      vi.unstubAllGlobals();
    });

    /** `config` claiming to belong to another product (no validation runs). */
    function foreign(config: ArmConfig): ArmConfig {
      const other: Product = { ...product, packageName: "@example/other-arm", displayName: "Other Arm" };
      return Object.assign(Object.create(ArmConfig.prototype) as ArmConfig, config, { product: other });
    }

    it("every arm built from one ArmConfig gets its own copy", async () => {
      const shared = fk.defaultConfig();
      const rig = new Rig({ arms: { a: new ArmSpec("gs_usb", { port: "gs_usb:1" }), b: new ArmSpec("gs_usb", { port: "gs_usb:2" }) } },
        product, { backends: fakeBackends() });
      const group = await rig.group(["all"], shared);
      const [a, b] = [group.get("a"), group.get("b")];
      expect(a.config).not.toBe(shared);
      expect(a.config).not.toBe(b.config);
      expect(a.config.joints).not.toBe(b.config.joints);
      expect(a.config.joints).toEqual(shared.joints);
      a.floatMode(); // the leader of arm-to-arm following: zero kp on its own config only
      expect(a.config.joints.every((j) => j.kp === 0)).toBe(true);
      expect(b.config.joints.map((j) => j.kp)).toEqual(shared.joints.map((j) => j.kp));
      expect(shared.joints.some((j) => j.kp > 0)).toBe(true);
    });

    it("a loader returning one cached ArmConfig still gives each arm its own", async () => {
      const cached = fk.defaultConfig();
      const rig = new Rig({
        arms: { a: new ArmSpec("gs_usb", { port: "gs_usb:1", profile: "p.yaml" }), b: new ArmSpec("gs_usb", { port: "gs_usb:2", profile: "p.yaml" }) },
      }, product, { backends: fakeBackends(), files: () => cached });
      const group = await rig.group();
      expect(group.get("a").config).not.toBe(group.get("b").config);
      expect(group.get("a").config).not.toBe(cached);
      group.get("b").floatMode();
      expect(group.get("a").config.joints.map((j) => j.kp)).toEqual(cached.joints.map((j) => j.kp));
    });

    it("a config of another product is refused", async () => {
      const other = foreign(fk.defaultConfig());
      const rig = new Rig({ arms: { a: new ArmSpec("gs_usb", { profile: "p.yaml" }) } }, product,
        { backends: fakeBackends(), files: () => other });
      await expect(rig.arm("a")).rejects.toThrow(RigError);
      await expect(rig.arm("a")).rejects.toThrow(/is a Other Arm profile; this rig's arms are/);
      await expect(rig.group(["a"], other)).rejects.toThrow(/Other Arm/);
    });

    it("explicit addresses are compared in one spelling", () => {
      expect(() => Rig.fromRaw({ arms: { a: { backend: "gs_usb", port: "ABC" }, b: { backend: "gs_usb", port: "gs_usb:ABC" } } }, product))
        .toThrow(/arms 'a' and 'b' use the same bus gs_usb:ABC/);
      expect(() => Rig.fromRaw({ arms: { a: { backend: "slcan", port: "serial:1A86:7523" }, b: { backend: "slcan", port: "serial:1a86:7523" } } },
        product)).toThrow(/same bus serial:1a86:7523/);
      expect(new ArmSpec("gs_usb", { port: "ABC" }).address).toBe("gs_usb:ABC");
    });

    it("an unnamed and a named entry resolving to the same adapter are refused before anything opens", async () => {
      const device = new FakeGsUsbDevice({ serialNumber: "ABC" });
      vi.stubGlobal("navigator", { usb: { getDevices: async () => [device] } });
      const rig = Rig.fromRaw({ arms: { a: { backend: "gs_usb" }, b: { backend: "gs_usb", port: "gs_usb:ABC" } } }, product);
      await expect(rig.group()).rejects.toThrow(/arm 'a' and arm 'b' resolve to the same adapter gs_usb:ABC/);
      expect(device.opened).toBe(false);
      expect((await rig.group(["b"])).length).toBe(1);
    });

    it("two device objects with one USB serial count as one adapter (Node resolvers)", async () => {
      const rig = Rig.fromRaw({ arms: { a: { backend: "gs_usb", port: "gs_usb:X1" }, b: { backend: "gs_usb", port: "gs_usb:X2" } } }, product, {
        backends: () => new GsUsbBackend(new FakeGsUsbDevice({ serialNumber: "SAME" })),
      });
      await expect(rig.group()).rejects.toThrow(RigError);
    });

    it("a leader arm on its follower's adapter is refused", async () => {
      const device = new FakeGsUsbDevice({ serialNumber: "ABC" });
      vi.stubGlobal("navigator", { usb: { getDevices: async () => [device] } });
      const rig = Rig.fromRaw({
        arms: { left: { backend: "gs_usb" }, spare: { backend: "gs_usb", port: "ABC" } },
        leaders: { left: { type: "arm", arm: "spare" } },
      }, product);
      const follower = await rig.arm("left");
      await expect(rig.leader("left", follower)).rejects.toThrow(/arm 'left' \(follower\) and arm 'spare' \(leader of 'left'\) resolve to the same adapter/);
      await expect(rig.leader("left", follower, { spare: follower })).rejects.toThrow(RigError);
    });

    it("a star leader on its follower's serial port is refused", async () => {
      const port = fakePort();
      const rig = Rig.fromRaw({
        arms: { left: { backend: "slcan", port: "serial:1a86:7523" } },
        leaders: { left: { type: "star", port: "serial:1a86:7523" } },
      }, product, { serialPorts: async () => port, files: () => starMap() });
      const follower = await rig.arm("left");
      expect(follower.backend).toBeInstanceOf(SlcanSerialBackend);
      await expect(rig.leader("left", follower)).rejects.toThrow(/arm 'left' \(follower\) and leaders.left \(star\) resolve to the same serial port/);
    });

    // ── ArmGroup ──
    function twoArms(clock = new VirtualClock()) {
      const a = makeArm(null, {}, clock);
      const b = makeArm(null, {}, clock);
      const motors: Record<string, M> = {};
      for (const [k, m] of Object.entries(a.fakes)) motors[`a${k}`] = m;
      for (const [k, m] of Object.entries(b.fakes)) motors[`b${k}`] = m;
      return { arms: new Map<string, Arm>([["a", a.arm], ["b", b.arm]]), motors };
    }

    it("group connects, enables and stops concurrently", async () => {
      const { arms, motors } = twoArms();
      let active = 0;
      let most = 0;
      for (const arm of arms.values()) {
        const original = arm.enable.bind(arm);
        arm.enable = async (options) => {
          most = Math.max(most, ++active);
          try {
            return await original(options);
          } finally {
            active--;
          }
        };
      }
      const group = new ArmGroup(arms, true, new RecordingLogger());
      await group.open();
      try {
        await group.enable();
        expect([...arms.values()].every((arm) => arm.state === "enabled")).toBe(true);
        expect(group.length).toBe(2);
        expect(group.get("a")).toBe(arms.get("a"));
        expect([...group].map(([n]) => n)).toEqual(["a", "b"]);
      } finally {
        await group.close();
      }
      expect(most).toBe(2); // both enables were in flight at once
      expect([...arms.values()].every((arm) => arm.state === "disconnected")).toBe(true);
      expect(Object.values(motors).some((m) => m.enabled)).toBe(false);
    });

    it("await using closes the group", async () => {
      const { arms, motors } = twoArms();
      {
        await using group = await new ArmGroup(arms, true, new RecordingLogger()).open();
        await group.enable();
      }
      expect([...arms.values()].every((arm) => arm.state === "disconnected")).toBe(true);
      expect(Object.values(motors).some((m) => m.enabled)).toBe(false);
    });

    it("group enable failure stops the others", async () => {
      const clock = new VirtualClock();
      const a = makeArm(null, {}, clock);
      const b = makeArm([new FakeMotor(1, undefined, { position: 0.3 }), new FakeMotor(2, undefined, { position: -0.5, temperature: 50.0 })],
        {}, clock);
      const group = new ArmGroup(new Map([["a", a.arm], ["b", b.arm]]), false, new RecordingLogger());
      await group.connect();
      const error = await group.enable().then(() => null, (e: unknown) => e);
      expect(error).toBeInstanceOf(ArmError);
      expect((error as Error).message).toMatch(/enable needs it below/);
      expect(a.arm.state).toBe("connected");
      expect(Object.values(a.fakes).some((m) => m.enabled)).toBe(false);
      await group.disconnect();
    });

    it("group connect failure disconnects everything", async () => {
      const clock = new VirtualClock();
      const a = makeArm(null, {}, clock);
      const b = makeArm([new FakeMotor(1, undefined, { position: 0.3 }), new FakeMotor(2, undefined, { online: false })], {}, clock);
      const group = new ArmGroup([a.arm, b.arm], true, new RecordingLogger());
      const error = await group.connect().then(() => null, (e: unknown) => e);
      expect(error).toBeInstanceOf(ArmError);
      expect((error as Error).message).toMatch(/no reply/);
      expect(a.arm.state).toBe("disconnected");
      expect(a.bus.isOpen).toBe(false);
    });

    it("a non-ArmError is wrapped with the arm's name, after every arm finished", async () => {
      const { arms } = twoArms();
      const logger = new RecordingLogger();
      const group = new ArmGroup(arms, true, logger);
      const finished: string[] = [];
      await expect(group.each(async (arm) => {
        const name = arm === arms.get("a") ? "a" : "b";
        await arm.clock.sleep(name === "a" ? 0.01 : 0.02);
        finished.push(name);
        throw new Error(`boom ${name}`);
      })).rejects.toThrow(/^a: boom a$/);
      expect(finished).toEqual(["a", "b"]);
      expect(logger.messages.some(([level, m]) => level === "error" && m.startsWith("several arms failed"))).toBe(true);
      expect(await group.each((arm) => arm.config.nJoints, ["b"])).toEqual({ b: 2 });
    });

    it("rig group runs concurrently only on separate buses", async () => {
      let rig = new Rig({ arms: { a: new ArmSpec("gs_usb", { port: "gs_usb:1" }), b: new ArmSpec("gs_usb", { port: "gs_usb:2" }) } },
        product, { backends: fakeBackends() });
      expect((await rig.group()).parallel).toBe(true);
      expect([...(await rig.group(["b"])).arms.keys()]).toEqual(["b"]);
      rig = new Rig({ arms: { a: new ArmSpec("gs_usb"), b: new ArmSpec("gs_usb") } }, product, { backends: fakeBackends() });
      expect((await rig.group()).parallel).toBe(false); // both "the only adapter"
    });

    it("rig and camera modules are product neutral", () => {
      const names = [product.packageName, product.packageName.split("/").pop()!, product.displayName,
        product.displayName.split(" ")[0]!, product.motorVendor].map((n) => n.toLowerCase());
      for (const file of ["../../src/rig.ts", "../../src/cameras.ts"]) {
        const text = readFileSync(new URL(file, import.meta.url), "utf8").toLowerCase();
        expect(names.filter((n) => text.includes(n)), file).toEqual([]);
      }
    });

    it("group close never releases an arm whose return was not verified", async () => {
      const { arm: a, fakes: fa } = makeArm([new FakeMotor(1, undefined, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, undefined, { position: -0.5, paramsSupported: true })]);
      a.stop = async () => false;
      const group = await new ArmGroup(new Map([["a", a]]), true, new RecordingLogger()).open();
      try {
        await a.enable();
      } finally {
        await group.close();
      }
      // only the gripper (release_on_fault) has a watchdog: detached, still holding
      expect(a.state).toBe("disconnected");
      expect(Object.values(fa).every((m) => m.enabled)).toBe(true);
      expect(Object.values(fa).some((m) => m.log.includes("disable"))).toBe(false);
    });

    it("group close keeps holding when a stiff joint would drop on its watchdog", async () => {
      const joints = twoJointConfig().joints.map((j) => j.replace({ canTimeoutMs: 200 }));
      const { arm: a, fakes: fa } = makeArm([new FakeMotor(1, undefined, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, undefined, { position: -0.5, paramsSupported: true })], { joints });
      a.stop = async () => false;
      const group = await new ArmGroup(new Map([["a", a]]), true, new RecordingLogger()).open();
      try {
        await a.enable();
      } finally {
        await group.close();
      }
      try {
        expect(a.state).toBe("enabled");
        expect(a["running"]).toBe(true);
        expect(fa[1]!.enabled).toBe(true);
      } finally {
        await a.disable();
      }
    });

    it("group close reports each arm", async () => {
      const { arms, motors } = twoArms();
      const group = new ArmGroup(arms, true, new RecordingLogger());
      await group.connect();
      await group.enable();
      arms.get("b")!.stop = async () => false;
      expect(await group.close()).toEqual({ a: "released", b: "detached" });
      expect(arms.get("a")!.state).toBe("disconnected");
      expect(arms.get("b")!.state).toBe("disconnected");
      expect(motors.a1!.enabled || motors.a2!.enabled).toBe(false);
      expect(motors.b1!.enabled).toBe(true);
    });

    it("group close reports an arm left energized by an earlier process", async () => {
      const clock = new VirtualClock();
      const a = makeArm([new FakeMotor(1, undefined, { position: 0.3, paramsSupported: true, enabled: true }),
        new FakeMotor(2, undefined, { position: -0.5, paramsSupported: true, enabled: true })], {}, clock);
      const b = makeArm([new FakeMotor(1, undefined, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, undefined, { position: -0.5, paramsSupported: true })], {}, clock);
      const group = new ArmGroup(new Map([["a", a.arm], ["b", b.arm]]), true, new RecordingLogger());
      await group.connect();
      expect(await group.close()).toEqual({ a: "energized", b: "released" });
      expect(a.fakes[1]!.enabled && a.fakes[2]!.enabled).toBe(true);
    });

    it("group close reports an unconfirmed release", async () => {
      const { arms, motors } = twoArms();
      const group = new ArmGroup(arms, true, new RecordingLogger());
      await group.connect();
      await group.enable();
      motors.a2!.online = false;
      // Python forces the return to rest to succeed; here stop() goes straight to the confirmed disable.
      const a = arms.get("a")!;
      a.stop = async () => {
        await a.disable();
        return true;
      };
      expect(await group.close()).toEqual({ a: "unknown", b: "released" });
      // never connected: nothing confirmed
      expect(await new ArmGroup(arms, true, new RecordingLogger()).close()).toEqual({ a: "unknown", b: "unknown" });
    });
  });
}
