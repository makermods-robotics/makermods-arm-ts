#!/usr/bin/env python3
"""Write the public API of a Python arm SDK as JSON, for the TypeScript packages to be checked against.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_api_manifest.py maker_arm   # contract/api_maker_arm.json
    ~/metal-arm-sdk/.venv/bin/python tools/gen_api_manifest.py metal_arm   # contract/api_metal_arm.json
    ... --check                                                            # exit 1 if the file is stale

Names are Python's (snake_case). A TypeScript test maps them to camelCase and fails when one is
missing, unless contract/api_exceptions.json lists it with a reason.
"""

import argparse
import ast
import dataclasses
import enum
import importlib
import inspect
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHARED_MODULES = ("arm", "bus", "config", "errors", "gravity", "gripper", "kinematics", "trajectory_ik", "testing",
                  "teleop", "mapping", "stall", "rig", "cameras", "leaders.base", "leaders.star", "leaders.arm",
                  "transport.base", "transport.mock", "transport.gs_usb", "transport.slcan_serial")
PRODUCT_MODULES = ("protocol", "motor", "_sim_motor", "_product")


def public(name: str) -> bool:
    return not name.startswith("_")


def describe_class(cls) -> dict:
    out = {"methods": {}, "properties": [], "fields": [], "constants": []}
    if isinstance(cls, type) and issubclass(cls, enum.Enum):
        out["constants"] = [m.name for m in cls]
        return out
    if dataclasses.is_dataclass(cls):
        out["fields"] = [f.name for f in dataclasses.fields(cls) if public(f.name)]
    for name in sorted(vars(cls)):
        if not public(name):
            continue
        raw = inspect.getattr_static(cls, name)
        if isinstance(raw, property):
            out["properties"].append(name)
        elif isinstance(raw, (staticmethod, classmethod)) or inspect.isfunction(raw):
            fn = raw.__func__ if isinstance(raw, (staticmethod, classmethod)) else raw
            params = [p for p in inspect.signature(fn).parameters if p not in ("self", "cls")]
            out["methods"][name] = params
        elif name.isupper():
            out["constants"].append(name)
    return out


def own_constants(module) -> list[str]:
    """UPPER_CASE names assigned at the top level of the module's own source (not imported ones)."""
    names = set()
    for node in ast.parse(inspect.getsource(module)).body:
        targets = node.targets if isinstance(node, ast.Assign) else [node.target] if isinstance(node, ast.AnnAssign) else []
        for target in targets:
            if isinstance(target, ast.Name) and public(target.id) and target.id.isupper():
                names.add(target.id)
    return sorted(names)


def describe_module(module) -> dict:
    out = {"classes": {}, "functions": {}, "constants": own_constants(module)}
    for name, value in sorted(vars(module).items()):
        if not public(name) or getattr(value, "__module__", None) != module.__name__:
            continue
        if inspect.isclass(value):
            out["classes"][name] = describe_class(value)
        elif inspect.isfunction(value):
            out["functions"][name] = list(inspect.signature(value).parameters)
    return out


def instance_attributes(package: str) -> list[str]:
    """Public attributes an Arm carries after construction (set in __init__, not on the class)."""
    testing = importlib.import_module(f"{package}.testing")
    arm, _ = testing.simulated_arm()
    return sorted(n for n in vars(arm) if public(n))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("package", choices=("maker_arm", "metal_arm"))
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    manifest = {"package": a.package, "shared": {}, "product": {}}
    for name in SHARED_MODULES:
        manifest["shared"][name] = describe_module(importlib.import_module(f"{a.package}.{name}"))
    for name in PRODUCT_MODULES:
        manifest["product"][name] = describe_module(importlib.import_module(f"{a.package}.{name}"))
    manifest["shared"]["arm"]["classes"]["Arm"]["attributes"] = instance_attributes(a.package)
    text = json.dumps(manifest, indent=1, sort_keys=True) + "\n"
    path = ROOT / "contract" / f"api_{a.package}.json"
    if a.check:
        if not path.exists() or path.read_text() != text:
            print("stale:", path.relative_to(ROOT))
            return 1
        return 0
    path.write_text(text)
    print("wrote", path.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
