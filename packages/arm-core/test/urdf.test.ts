import { describe, expect, it } from "vitest";
import { ValueError } from "../src/errors.js";
import { xyzRpyToPose } from "../src/kinematics.js";
import { parseXml, pyFloat, readUrdf } from "../src/urdf.js";
import { ARMS, armFixture, compare, type Json } from "./helpers/math-fixtures.js";

describe("XML reader", () => {
  it("reads elements, attributes and nesting, skipping prolog, comments, PIs, DOCTYPE and CDATA", () => {
    const root = parseXml(`<?xml version="1.0" encoding="utf-8"?>
      <!-- a comment with <tags> and "quotes" -->
      <!DOCTYPE robot [ <!ENTITY x "y"> ]>
      <robot name='r'>
        text is ignored
        <link name="a"/>
        <?pi inside?>
        <link
          name="b"
        ><inertial><mass value="1.5" /></inertial></link>
        <![CDATA[ <joint name="not-a-joint"/> ]]>
        <joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
      </robot>`);
    expect(root.tag).toBe("robot");
    expect(root.get("name")).toBe("r");
    expect(root.findAll("link").map((l) => l.get("name"))).toEqual(["a", "b"]);
    expect(root.find("link")!.get("name")).toBe("a");
    expect(root.findAll("joint")).toHaveLength(1);
    expect(root.find("link")!.find("inertial")).toBeNull();
    expect(root.findAll("link")[1]!.find("inertial")!.find("mass")!.get("value")).toBe("1.5");
    expect(root.get("missing")).toBeNull();
    expect(root.get("missing", "-inf")).toBe("-inf");
  });

  it("decodes entities and normalizes whitespace in attribute values", () => {
    const root = parseXml(`<r a="&lt;&amp;&gt;&quot;&apos;&#65;&#x42;" b="1\t2\n3"/>`);
    expect(root.get("a")).toBe(`<&>"'AB`);
    expect(root.get("b")).toBe("1 2 3");
  });

  it.each([
    ["<a><b></a>", /mismatched/],
    ["<a>", /unclosed/],
    ["<a/><b/>", /more than one root/],
    ["x<a/>", /outside the root/],
    ['<a b="&nope;"/>', /undefined entity/],
    ['<a b="1" b="2"/>', /duplicate attribute/],
    ["<a b=1/>", /not quoted/],
    ["<!-- only a comment -->", /no root element/],
    ["<a><!-- open", /unterminated/],
  ])("refuses malformed XML %s", (text, message) => {
    expect(() => parseXml(text)).toThrow(ValueError);
    expect(() => parseXml(text)).toThrow(message);
  });
});

describe("pyFloat", () => {
  it("reads what Python's float() reads", () => {
    expect(pyFloat("1e-3")).toBe(0.001);
    expect(pyFloat(" 2 ")).toBe(2);
    expect(pyFloat("-.5")).toBe(-0.5);
    expect(pyFloat("+3.")).toBe(3);
    expect(pyFloat("1_000.5")).toBe(1000.5);
    expect(pyFloat("inf")).toBe(Infinity);
    expect(pyFloat("-Infinity")).toBe(-Infinity);
    expect(pyFloat("NaN")).toBeNaN();
    expect(pyFloat("-2.4247119e-06")).toBe(-2.4247119e-6);
  });

  it.each(["", "abc", "1.2.3", "0x10", "1e", "_1"])("refuses %j", (text) => {
    expect(() => pyFloat(text)).toThrow(/could not convert/);
  });
});

describe("URDF", () => {
  it("defaults: no origin, no axis, no limit, continuous joints", () => {
    const { links, joints } = readUrdf(`<robot>
      <link name="a"/><link name="b"><inertial><mass value="2"/></inertial></link><link name="c"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
      <joint name="k" type="continuous"><parent link="b"/><child link="c"/><axis xyz="0 0 2"/>
        <limit lower="-1" upper="1"/></joint>
    </robot>`);
    expect(links.get("a")).toEqual({ mass: 0, com: [0, 0, 0] });
    expect(links.get("b")).toEqual({ mass: 2, com: [0, 0, 0] });
    const j = joints.get("j")!;
    expect([j.xyz, j.rpy, j.axis, j.lower, j.upper]).toEqual([[0, 0, 0], [0, 0, 0], [1, 0, 0], -Infinity, Infinity]);
    const k = joints.get("k")!;
    expect([k.axis, k.lower, k.upper]).toEqual([[0, 0, 1], -Infinity, Infinity]);
  });

  it("refuses a zero axis and a joint without parent or child", () => {
    const joint = (body: string) => `<robot><link name="a"/><link name="b"/><joint name="j" type="revolute">${body}</joint></robot>`;
    expect(() => readUrdf(joint('<parent link="a"/><child link="b"/><axis xyz="0 0 0"/>'))).toThrow(/zero axis/);
    expect(() => readUrdf(joint('<parent link="a"/>'))).toThrow(/parent and a child/);
    expect(() => readUrdf(joint('<parent link="a"/><child link="b"/><origin xyz="1 2"/>'))).toThrow(/3 values/);
  });

  for (const arm of ARMS) {
    it(`${arm}: reads the shipped URDF exactly as the Python SDK does`, () => {
      const fx: Json = armFixture(arm);
      const { links, joints } = readUrdf(fx.profile.urdf);
      expect([...links.keys()]).toEqual(fx.urdf_parsed.links.map((l: Json) => l[0]));
      for (const [name, mass, com] of fx.urdf_parsed.links) {
        const link = links.get(name)!;
        expect(Object.is(link.mass, mass)).toBe(true);
        expect(compare([...link.com], com).ulps).toBe(0);
      }
      expect([...joints.keys()]).toEqual(fx.urdf_parsed.joints.map((j: Json) => j.name));
      for (const want of fx.urdf_parsed.joints) {
        const j = joints.get(want.name)!;
        expect([j.type, j.parent, j.child]).toEqual([want.type, want.parent, want.child]);
        expect(compare([...j.axis], want.axis).ulps).toBe(0);
        expect(compare([j.lower, j.upper], [want.lower, want.upper]).ulps).toBe(0);
        expect(compare(xyzRpyToPose(j.xyz, j.rpy), want.origin).ulps).toBe(0);
      }
    });
  }
});
