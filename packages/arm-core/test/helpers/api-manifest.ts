/**
 * Checks a TypeScript arm package against the Python SDK's public API (contract/api_<package>.json,
 * written by tools/gen_api_manifest.py): every Python name must exist in camelCase, unless
 * contract/api_exceptions.json lists it with a reason.
 */

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

interface ClassEntry {
  methods: Record<string, string[]>;
  properties: string[];
  fields: string[];
  constants: string[];
  attributes?: string[];
}

interface ModuleEntry {
  classes: Record<string, ClassEntry>;
  functions: Record<string, string[]>;
  constants: string[];
}

interface Manifest {
  package: string;
  shared: Record<string, ModuleEntry>;
  product: Record<string, ModuleEntry>;
}

export interface ModuleBinding {
  /** Where the module's functions, classes and constants are exported in TypeScript. */
  exports: Record<string, unknown>;
  /** Python class name -> TypeScript export name, where they differ. */
  rename?: Record<string, string>;
  /** Python class name -> an instance, to check instance fields and attributes on. */
  instances?: Record<string, object>;
  /** The module's constants are camelCase fields of `exports` (the product object). */
  constantsAsFields?: boolean;
}

const contract = (name: string): string => fileURLToPath(new URL(`../../../../contract/${name}`, import.meta.url));

export const camel = (name: string): string => name.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase());

function has(target: unknown, name: string): boolean {
  for (let o = target as object | null; o; o = Object.getPrototypeOf(o)) {
    if (Object.getOwnPropertyDescriptor(o, name)) return true;
  }
  return false;
}

/** Python names with no TypeScript counterpart that are not listed as exceptions, as "module.Class.name". */
export function missingFromTypeScript(
  pythonPackage: "maker_arm" | "metal_arm",
  bindings: Record<string, ModuleBinding>,
): { missing: string[]; unusedExceptions: string[] } {
  const manifest = JSON.parse(readFileSync(contract(`api_${pythonPackage}.json`), "utf8")) as Manifest;
  const listed = JSON.parse(readFileSync(contract("api_exceptions.json"), "utf8")) as Record<string, Record<string, string>>;
  const exceptions = { ...listed.all, ...listed[pythonPackage] };
  const used = new Set<string>();
  const missing: string[] = [];
  const note = (path: string, present: boolean) => {
    const excepted = Object.keys(exceptions).find((e) => path === e || path.startsWith(`${e}.`));
    if (excepted) used.add(excepted);
    else if (!present) missing.push(path);
  };
  const modules = { ...manifest.shared, ...manifest.product };
  for (const [moduleName, module] of Object.entries(modules)) {
    const binding = bindings[moduleName];
    if (!binding) {
      note(moduleName, false);
      continue;
    }
    for (const fn of Object.keys(module.functions)) note(`${moduleName}.${fn}`, camel(fn) in binding.exports);
    for (const constant of module.constants) {
      const name = binding.constantsAsFields ? camel(constant.toLowerCase()) : constant;
      note(`${moduleName}.${constant}`, name in binding.exports);
    }
    for (const [className, entry] of Object.entries(module.classes)) {
      const exported = binding.exports[binding.rename?.[className] ?? className] as { prototype?: object } | undefined;
      const instance = binding.instances?.[className];
      const path = `${moduleName}.${className}`;
      if (exported === undefined && instance === undefined) {
        note(path, false);
        continue;
      }
      const targets = [exported, exported?.prototype, instance];
      const present = (name: string) => targets.some((t) => t && has(t, name));
      for (const method of Object.keys(entry.methods)) note(`${path}.${method}`, present(camel(method)));
      for (const property of entry.properties) note(`${path}.${property}`, present(camel(property)));
      for (const constant of entry.constants) note(`${path}.${constant}`, present(constant));
      if (instance) {
        for (const field of [...entry.fields, ...(entry.attributes ?? [])]) note(`${path}.${field}`, present(camel(field)));
      }
    }
  }
  const unusedExceptions = Object.keys(exceptions).filter((e) => !used.has(e));
  return { missing: missing.sort(), unusedExceptions };
}
