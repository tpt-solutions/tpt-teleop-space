#!/usr/bin/env python3
"""Call-graph recursion audit (spec §2 / Phase 21).

The bash scan in no-recursion.sh catches direct self-recursion only. This
tool builds a call graph over each crate's library functions and reports
every cycle, including mutual recursion (a → b → a) and longer loops.

Nodes are scoped the way Rust scopes names:

* an inherent method is `Type::name`,
* a trait-impl method is `Trait@Type::name`, so that delegation such as
  `impl Transition for T { fn next(..) { T::next(..) } }` is not a loop,
* a free function is `name`.

Resolution follows Rust's path rules. `Type::name(` and `Self::name(` /
`self.name(` (inside an impl) prefer the inherent method and fall back to a
trait-impl method of that type. A bare `name(` reaches only free functions,
since methods cannot be called bare. Calls through another receiver
(`x.name(`) are not attributed, because the receiver's type needs a type
checker. Test modules are skipped. Exits 1 when any cycle is found.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
FN_DEF = re.compile(r"\bfn\s+([A-Za-z_]\w*)")
IMPL_TRAIT = re.compile(r"^\s*impl\b(?:<.*?>)?\s+([\w:]+)(?:<[^>]*>)?\s+for\s+([A-Z]\w*)")
IMPL_INHERENT = re.compile(r"^\s*impl\b(?:<.*?>)?\s+([A-Z]\w*)")
BARE = re.compile(r"(?<![\w:.])([A-Za-z_]\w*)\s*\(")
QUALIFIED = re.compile(r"\b([A-Z]\w*)::([A-Za-z_]\w*)\s*\(")
SELF = re.compile(r"(?:\bSelf::|\bself\.)([A-Za-z_]\w*)\s*\(")
# \x5c is a backslash, spelled out so the escape survives any copy of this file.
STRING = re.compile(r'"(?:\x5c.|[^"\x5c])*"')
CHAR = re.compile(r"'(?:\x5c.|[^'\x5c])'")


def strip_noise(line):
    """Drop comments and literal contents so braces and calls inside them do not count."""
    line = line.split("//", 1)[0]
    line = STRING.sub('""', line)
    return CHAR.sub("''", line)


def fn_body(lines, start):
    """The brace-balanced body of the fn defined on `lines[start]`, or None if bodiless."""
    depth = 0
    opened = False
    body = []
    for j in range(start, len(lines)):
        clean = strip_noise(lines[j])
        if not opened:
            if ";" in clean and "{" not in clean:
                return None
            if "{" not in clean:
                continue
            opened = True
        depth += clean.count("{") - clean.count("}")
        if j > start:
            body.append(clean)
        if depth <= 0:
            break
    return "\n".join(body)


def crate_functions(crate_src):
    """Return (defs, bodies): node → location, and (node, impl type, body) per fn."""
    defs = {}
    bodies = []
    for path in sorted(crate_src.rglob("*.rs")):
        rel = path.relative_to(ROOT).as_posix()
        lines = path.read_text(encoding="utf-8").splitlines()
        cut = next((i for i, line in enumerate(lines) if "mod tests" in line), len(lines))
        lines = lines[:cut]
        impl_type, trait = None, None
        for i, line in enumerate(lines):
            if line.startswith("}"):
                impl_type, trait = None, None
            if not line.lstrip().startswith("//"):
                head = IMPL_TRAIT.match(line)
                if head:
                    trait, impl_type = head.group(1), head.group(2)
                else:
                    head = IMPL_INHERENT.match(line)
                    if head:
                        trait, impl_type = None, head.group(1)
            m = FN_DEF.search(line)
            if not m or line.lstrip().startswith("//"):
                continue
            body = fn_body(lines, i)
            if body is None:
                continue
            name = m.group(1)
            if impl_type and trait:
                node = f"{trait}@{impl_type}::{name}"
            elif impl_type:
                node = f"{impl_type}::{name}"
            else:
                node = name
            defs.setdefault(node, f"{rel}:{i + 1}")
            bodies.append((node, impl_type, body))
    return defs, bodies


def lookup(type_name, method, defs):
    """Rust path resolution for `Type::method`: the inherent method, else a trait-impl method."""
    inherent = f"{type_name}::{method}"
    if inherent in defs:
        return inherent
    suffix = f"@{type_name}::{method}"
    return next((node for node in defs if node.endswith(suffix)), None)


def resolve(impl_type, body, defs):
    """The defined nodes that `body` calls."""
    out = set()
    for m in QUALIFIED.finditer(body):
        target = lookup(m.group(1), m.group(2), defs)
        if target:
            out.add(target)
    if impl_type:
        for m in SELF.finditer(body):
            target = lookup(impl_type, m.group(1), defs)
            if target:
                out.add(target)
    for m in BARE.finditer(body):
        if m.group(1) in defs:  # a free function; methods are never called bare
            out.add(m.group(1))
    return out


def cycles(graph):
    """Tarjan's SCC: the strongly connected components that contain a cycle."""
    index, low, on_stack, stack, out = {}, {}, set(), [], []
    counter = [0]

    def visit(v):
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        on_stack.add(v)
        for w in graph.get(v, ()):
            if w not in index:
                visit(w)
                low[v] = min(low[v], low[w])
            elif w in on_stack:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                on_stack.discard(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) > 1 or v in graph.get(v, ()):
                out.append(sorted(comp))

    for v in list(graph):
        if v not in index:
            visit(v)
    return out


def main():
    failed = False
    for crate_src in sorted((ROOT / "crates").glob("*/src")):
        crate = crate_src.parent.name
        defs, bodies = crate_functions(crate_src)
        graph = {}
        for node, impl_type, body in bodies:
            graph.setdefault(node, set()).update(resolve(impl_type, body, defs))
        for comp in cycles(graph):
            failed = True
            print(f"CYCLE in {crate} at {defs[comp[0]]}: {' -> '.join(comp)} -> {comp[0]}")
    if failed:
        print("FAIL: recursion cycles found in the call graph.")
        return 1
    print("PASS: no call-graph cycles (direct or mutual) in library source.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
