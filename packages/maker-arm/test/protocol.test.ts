// Port of maker-arm-sdk tests/unit/test_protocol.py.
import { describe, expect, it } from "vitest";
import { protocol as p, ValueError } from "@makermods/maker-arm";
import { feedbackBytes } from "./fakes.js";

/** pytest.approx: |a - b| <= max(rel * |b|, abs); abs alone disables rel. */
function close(actual: number, expected: number, opts: { abs?: number; rel?: number } = {}): boolean {
  const tol = opts.abs !== undefined && opts.rel === undefined
    ? opts.abs : Math.max((opts.rel ?? 1e-6) * Math.abs(expected), opts.abs ?? 1e-12);
  return Math.abs(actual - expected) <= tol;
}

const fromHex = (hex: string): Uint8Array => Uint8Array.from(hex.match(/../g)!.map((b) => parseInt(b, 16)));

/** struct.pack("<H2xI", index, value). */
function paramBytes(index: number, value: number): Uint8Array {
  const out = new Uint8Array(8);
  const view = new DataView(out.buffer);
  view.setUint16(0, index, true);
  view.setUint32(4, value, true);
  return out;
}

/** Python's round(): ties to even. */
function pyRound(x: number): number {
  const f = Math.floor(x);
  const d = x - f;
  if (d !== 0.5) return Math.round(x);
  return f % 2 === 0 ? f : f + 1;
}

const feedback = (msg: ReturnType<typeof p.parseFrame>): p.MotorFeedback => {
  expect(msg?.kind).toBe("feedback");
  return msg as p.MotorFeedback;
};

describe("protocol", () => {
  it.each([
    ["encodeEnable", p.encodeEnable, "ffffffffffffffFC"],
    ["encodeDisable", p.encodeDisable, "ffffffffffffffFD"],
    ["encodeClearFault", p.encodeClearFault, "ffffffffffffffFB"],
    ["encodeFaultQuery", p.encodeFaultQuery, "ffffffffffff00FB"],
    ["encodeSetZero", p.encodeSetZero, "ffffffffffffffFE"],
  ] as const)("commands are standard frames to the motor id: %s", (_name, encode, payload) => {
    expect(encode(7)).toEqual([7, fromHex(payload)]);
  });

  it.each([0, 128])("motor id range: %i", (bad) => {
    expect(() => p.encodeEnable(bad)).toThrow(ValueError);
  });

  it("control packing matches lerobot layout", () => {
    const [cid, data] = p.encodeControl(3, 0.0, 0.0, 500.0, 5.0, 0.0, p.RS02);
    expect(cid).toBe(3);
    const q = (data[0]! << 8) | data[1]!;
    const kp = ((data[3]! & 0xf) << 8) | data[4]!;
    const kd = (data[5]! << 4) | (data[6]! >> 4);
    expect(q).toBe(pyRound(12.57 * 65535 / 25.14));
    expect(kp === 4095 && kd === 4095).toBe(true);
  });

  it("control clamps and rejects non finite", () => {
    const [, hi] = p.encodeControl(1, 99.0, 0, 0, 0, 0);
    expect(hi.subarray(0, 2)).toEqual(Uint8Array.from([0xff, 0xff]));
    expect(() => p.encodeControl(1, NaN, 0, 0, 0, 0)).toThrow(ValueError);
  });

  it("feedback decode masks status bits out of temperature", () => {
    const data = feedbackBytes(5, { pos: 1.0, temp: 41.3, mode: 2, fault: false, warn: true });
    const fb = feedback(p.parseFrame(p.HOST_CAN_ID, data));
    expect(fb.motorId).toBe(5);
    expect(close(fb.position, 1.0, { abs: 1e-3 })).toBe(true);
    expect(close(fb.temperature, 41.3)).toBe(true);
    expect(fb.mode === 2 && fb.warningFlag && !fb.faultFlag).toBe(true);
  });

  it("old feedback reads mode zero and true temperature", () => {
    const data = Uint8Array.from(feedbackBytes(1, { temp: 35.0 }));
    expect(data[6]! >> 4).toBe(0);
    const fb = feedback(p.parseFrame(p.HOST_CAN_ID, data));
    expect(fb.temperature === 35.0 && fb.mode === 0).toBe(true);
  });

  it("model scaling differs", () => {
    const data = feedbackBytes(2, { tau: 10.0, params: p.RS02 });
    expect(close(feedback(p.parseFrame(p.HOST_CAN_ID, data, { params: p.RS02 })).torque, 10.0, { abs: 0.01 })).toBe(true);
    expect(close(feedback(p.parseFrame(p.HOST_CAN_ID, data, { params: p.RS00 })).torque, 10.0, { abs: 0.5 })).toBe(false);
  });

  it.each([
    ["five bytes", Uint8Array.from([7, 1, 0, 0, 0])],
    ["padded eight bytes", Uint8Array.from([7, 1, 0, 0, 0, 0, 0, 0])],
  ])("fault status five or padded eight bytes: %s", (_name, data) => {
    const st = p.parseFrame(p.HOST_CAN_ID, data);
    expect(st?.kind).toBe("status");
    const status = st as p.FaultStatus;
    expect(status.motorId === 7 && status.word === 1 && status.hasFault).toBe(true);
    expect(status.text).toBe("motor overtemperature");
  });

  it("fault text decodes bits and reports unknown bits", () => {
    expect(p.faultText(0)).toBe("no fault");
    expect(p.faultText(null)).toBe("unknown");
    expect(p.faultText((1 << 2) | (1 << 14))).toBe("undervoltage, stall/overload");
    expect(p.faultText(1 << 6)).toBe("bit6");
  });

  it("param replies are filtered by arbitration id", () => {
    // data[0] is the register index low byte (0x28) and must not be mistaken for a motor ID.
    const reply = paramBytes(p.ParamIndex.CAN_TIMEOUT, 10000);
    const msg = p.parseFrame(0x300 | 7, reply);
    expect(msg).toBeInstanceOf(p.ParamReply);
    const param = msg as p.ParamReply;
    expect(param.motorId === 7 && param.value("u32") === 10000).toBe(true);
    expect(p.replyMotorId(0x300 | 7, reply)).toBe(7);
    expect(p.parseFrame(0x400 | 7, reply)!.motorId).toBe(7);
  });

  it("extended and foreign frames are ignored", () => {
    const data = feedbackBytes(1);
    expect(p.parseFrame(p.HOST_CAN_ID, data, { extended: true })).toBeNull();
    expect(p.parseFrame(0x123, data)).toBeNull();
    expect(p.replyMotorId(0x02fd01, data, true)).toBeNull();
  });

  it("write param layout", () => {
    const [cid, data] = p.encodeWriteParam(7, p.ParamIndex.CAN_TIMEOUT, 10000, "u32");
    expect(cid).toBe(0x407);
    expect(data).toEqual(paramBytes(0x7028, 10000));
    const read = new Uint8Array(8); // struct.pack("<H6x", 0x7005)
    new DataView(read.buffer).setUint16(0, 0x7005, true);
    expect(p.encodeReadParam(7, 0x7005)).toEqual([0x307, read]);
  });
});
