/**
 * URDF reading without a DOM: a minimal XML reader (elements, attributes, comments, processing
 * instructions, CDATA and DOCTYPE skipped, the predefined and numeric entities), enough for URDF, so
 * it runs in Node, browsers and Web Workers alike. Lookups follow Python's ElementTree: `find` and
 * `findAll` search direct children by tag.
 */

import { ValueError } from "./errors.js";
import { norm } from "./linalg.js";

/** One XML element: tag, attributes and child elements (text is dropped). */
export class XmlElement {
  readonly children: XmlElement[] = [];

  constructor(readonly tag: string, readonly attrs: Readonly<Record<string, string>>) {}

  /** The first direct child with this tag, or null. */
  find(tag: string): XmlElement | null {
    return this.children.find((c) => c.tag === tag) ?? null;
  }

  /** Every direct child with this tag, in document order. */
  findAll(tag: string): XmlElement[] {
    return this.children.filter((c) => c.tag === tag);
  }

  /** An attribute's value, or `fallback`. */
  get(name: string): string | null;
  get(name: string, fallback: string): string;
  get(name: string, fallback: string | null = null): string | null {
    return Object.hasOwn(this.attrs, name) ? this.attrs[name]! : fallback;
  }
}

const NAME = /[A-Za-z_:][-A-Za-z0-9_:.]*/y;
const SPACE = /\s*/y;

function decodeEntities(text: string, at: number): string {
  return text.replace(/&(#x[0-9A-Fa-f]+|#[0-9]+|[A-Za-z]+);|&/g, (whole, body: string | undefined) => {
    if (body === undefined) throw new ValueError(`URDF: XML parse error: stray '&' near offset ${at}`);
    if (body[0] === "#") {
      const code = body[1] === "x" ? parseInt(body.slice(2), 16) : parseInt(body.slice(1), 10);
      return String.fromCodePoint(code);
    }
    const named: Record<string, string> = { amp: "&", lt: "<", gt: ">", quot: '"', apos: "'" };
    const value = named[body];
    if (value === undefined) throw new ValueError(`URDF: XML parse error: undefined entity ${whole} near offset ${at}`);
    return value;
  });
}

/** Parse an XML document into its root element. Throws ValueError on malformed XML. */
export function parseXml(text: string): XmlElement {
  let i = 0;
  const n = text.length;
  const fail = (what: string): never => {
    const line = text.slice(0, i).split("\n").length;
    throw new ValueError(`URDF: XML parse error: ${what} (line ${line})`);
  };
  const skip = (open: string, close: string): void => {
    const end = text.indexOf(close, i + open.length);
    if (end < 0) fail(`unterminated ${open}`);
    i = end + close.length;
  };
  const readName = (): string => {
    NAME.lastIndex = i;
    const m = NAME.exec(text);
    if (!m) return fail("expected a name");
    i = NAME.lastIndex;
    return m[0];
  };
  const space = (): void => {
    SPACE.lastIndex = i;
    SPACE.exec(text);
    i = SPACE.lastIndex;
  };

  let root: XmlElement | null = null;
  const stack: XmlElement[] = [];
  while (i < n) {
    const lt = text.indexOf("<", i);
    if (lt < 0) {
      if (text.slice(i).trim() && !stack.length) fail("text outside the root element");
      i = n;
      break;
    }
    if (!stack.length && text.slice(i, lt).trim()) fail("text outside the root element");
    i = lt;
    if (text.startsWith("<!--", i)) skip("<!--", "-->");
    else if (text.startsWith("<?", i)) skip("<?", "?>");
    else if (text.startsWith("<![CDATA[", i)) {
      if (!stack.length) fail("CDATA outside the root element");
      skip("<![CDATA[", "]]>");
    } else if (text.startsWith("<!DOCTYPE", i)) {
      // Skip to the closing '>' of the declaration, past an internal subset in brackets.
      let depth = 0;
      for (i += 9; i < n; i++) {
        const ch = text[i];
        if (ch === "[") depth++;
        else if (ch === "]") depth--;
        else if (ch === ">" && depth <= 0) break;
      }
      if (i >= n) fail("unterminated <!DOCTYPE");
      i++;
    } else if (text.startsWith("</", i)) {
      i += 2;
      const tag = readName();
      space();
      if (text[i] !== ">") fail(`malformed end tag </${tag}`);
      i++;
      const open = stack.pop();
      if (!open || open.tag !== tag) fail(`mismatched tag </${tag}>`);
    } else {
      i++;
      const tag = readName();
      const attrs: Record<string, string> = {};
      for (;;) {
        space();
        if (text.startsWith("/>", i) || text[i] === ">") break;
        if (i >= n) fail(`unterminated <${tag}`);
        const name = readName();
        space();
        if (text[i] !== "=") fail(`attribute ${name} without a value`);
        i++;
        space();
        const quote = text[i];
        if (quote !== '"' && quote !== "'") fail(`attribute ${name} value not quoted`);
        const end = text.indexOf(quote!, i + 1);
        if (end < 0) fail(`unterminated attribute ${name}`);
        const raw = text.slice(i + 1, end);
        if (raw.includes("<")) fail(`'<' in attribute ${name}`);
        if (Object.hasOwn(attrs, name)) fail(`duplicate attribute ${name}`);
        // XML attribute-value normalization: each whitespace character becomes a space.
        attrs[name] = decodeEntities(raw.replace(/[\t\n\r]/g, " "), i);
        i = end + 1;
      }
      const element = new XmlElement(tag, attrs);
      if (stack.length) stack[stack.length - 1]!.children.push(element);
      else if (root) fail("more than one root element");
      else root = element;
      if (text.startsWith("/>", i)) i += 2;
      else {
        i++;
        stack.push(element);
      }
    }
  }
  if (stack.length) fail(`unclosed <${stack[stack.length - 1]!.tag}>`);
  if (!root) return fail("no root element");
  return root;
}

/** Python's float() of a string: decimal or exponent notation, inf / nan, surrounding whitespace. */
export function pyFloat(text: string): number {
  const s = text.trim().toLowerCase();
  const special = /^([+-]?)(inf|infinity|nan)$/.exec(s);
  if (special) return special[2] === "nan" ? NaN : special[1] === "-" ? -Infinity : Infinity;
  const plain = s.replace(/(?<=\d)_(?=\d)/g, "");
  if (/^[+-]?(\d+\.?\d*|\.\d+)(e[+-]?\d+)?$/.test(plain)) return Number(plain);
  throw new ValueError(`could not convert string to float: '${text}'`);
}

function floats(text: string | null, fallback: readonly number[]): number[] {
  if (!text) return [...fallback];
  return text.split(/\s+/).filter((t) => t !== "").map(pyFloat);
}

/** A URDF link: what gravity needs. */
export interface UrdfLink {
  readonly mass: number;
  /** Centre of mass in the link frame. */
  readonly com: readonly number[];
}

/** A URDF joint, as written (origin as xyz / rpy, unit axis). */
export interface UrdfJoint {
  readonly name: string;
  readonly type: string;
  readonly parent: string;
  readonly child: string;
  readonly xyz: readonly number[];
  readonly rpy: readonly number[];
  /** Unit, joint frame. */
  readonly axis: readonly number[];
  readonly lower: number;
  readonly upper: number;
}

function required(node: XmlElement, name: string, what: string): string {
  const value = node.get(name);
  if (value === null) throw new ValueError(`URDF: ${what} has no ${name}`);
  return value;
}

function checkTriple(values: number[], what: string): number[] {
  if (values.length !== 3) throw new ValueError(`URDF: ${what} needs 3 values, got ${values.length}`);
  return values;
}

/** Links and joints of a URDF document, by name in document order. */
export function readUrdf(text: string): { links: Map<string, UrdfLink>; joints: Map<string, UrdfJoint> } {
  const root = parseXml(text);
  const links = new Map<string, UrdfLink>();
  for (const node of root.findAll("link")) {
    const name = required(node, "name", "a link");
    const inertial = node.find("inertial");
    let mass = 0.0;
    let com = [0, 0, 0];
    if (inertial !== null) {
      const m = inertial.find("mass");
      mass = m !== null ? pyFloat(required(m, "value", `link ${name} mass`)) : 0.0;
      const origin = inertial.find("origin");
      if (origin !== null) {
        // The COM is the only thing gravity needs; the inertial frame's rpy only turns the tensor.
        com = checkTriple(floats(origin.get("xyz"), [0, 0, 0]), `link ${name} inertial origin`);
      }
    }
    links.set(name, { mass, com });
  }
  const joints = new Map<string, UrdfJoint>();
  for (const node of root.findAll("joint")) {
    const name = required(node, "name", "a joint");
    const origin = node.find("origin");
    const xyz = checkTriple(floats(origin !== null ? origin.get("xyz") : null, [0, 0, 0]), `joint ${name} origin`);
    const rpy = checkTriple(floats(origin !== null ? origin.get("rpy") : null, [0, 0, 0]), `joint ${name} origin`);
    const axisNode = node.find("axis");
    const axis = checkTriple(floats(axisNode !== null ? axisNode.get("xyz") : null, [1, 0, 0]), `joint ${name} axis`);
    const length = norm(axis);
    if (length === 0) throw new ValueError(`URDF: joint ${name} has a zero axis`);
    const limit = node.find("limit");
    const kind = required(node, "type", `joint ${name}`);
    const lower = limit !== null && kind !== "continuous" ? pyFloat(limit.get("lower", "-inf")) : -Infinity;
    const upper = limit !== null && kind !== "continuous" ? pyFloat(limit.get("upper", "inf")) : Infinity;
    const parent = node.find("parent");
    const child = node.find("child");
    if (parent === null || child === null) throw new ValueError(`URDF: joint ${name} needs a parent and a child`);
    joints.set(name, {
      name, type: kind,
      parent: required(parent, "link", `joint ${name} parent`),
      child: required(child, "link", `joint ${name} child`),
      xyz, rpy, axis: axis.map((v) => v / length), lower, upper,
    });
  }
  return { links, joints };
}
