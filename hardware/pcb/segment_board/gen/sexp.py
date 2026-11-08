"""Minimal S-expression reader/writer for KiCad files."""

import re

_TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')


class Q(str):
    """A string that must be written quoted."""


def parse(text):
    stack = [[]]
    for tok in _TOKEN.findall(text):
        if tok == "(":
            stack.append([])
        elif tok == ")":
            node = stack.pop()
            stack[-1].append(node)
        elif tok.startswith('"'):
            stack[-1].append(Q(tok[1:-1].replace('\\"', '"').replace("\\\\", "\\")))
        else:
            stack[-1].append(tok)
    return stack[0][0]


def _atom(x):
    if isinstance(x, Q):
        return '"' + x.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(x, float):
        s = f"{x:.4f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(x)


def dump(node, indent=0):
    """Write node keeping element order; nested lists with children go on their own lines."""
    if not isinstance(node, list):
        return _atom(node)
    simple = all(not isinstance(c, list) or all(not isinstance(cc, list) for cc in c) for c in node)
    if simple and len(node) <= 6:
        return "(" + " ".join(dump(c) for c in node) + ")"
    pad = "\t" * indent
    out = "("
    for i, c in enumerate(node):
        if isinstance(c, list):
            out += "\n" + pad + "\t" + dump(c, indent + 1)
        else:
            out += ("" if i == 0 else " ") + _atom(c)
    return out + "\n" + pad + ")"


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def first(node, key):
    r = find(node, key)
    return r[0] if r else None
