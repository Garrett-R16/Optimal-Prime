"""Lossless s-expression reader/writer for KiCad board files.

An s-expression is a file *syntax* -- nested parentheses, atoms inside. It carries no
geometric meaning of its own; ``.kicad_pcb`` merely happens to be written in it.

The design constraint that drives this module is losslessness. Anything that parses a board
into a semantic model and regenerates it will silently drop fields belonging to a KiCad
version it does not know about. So we parse into a *token tree* whose leaves keep their exact
source text, mutate only the nodes we care about, and re-serialise everything else verbatim.

Round-trip guarantee: ``parse(dumps(parse(x))) == parse(x)``. Byte-identity with the original
is deliberately *not* claimed -- KiCad reformats on save too -- but the token tree is a
fixpoint after one pass, which is what makes reruns diffable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Sequence, Union

__all__ = ["Atom", "Node", "SExprError", "parse", "dumps", "head", "find", "find_all",
           "atoms", "sym", "quoted", "num"]


class SExprError(ValueError):
    """Raised on malformed s-expression input."""


@dataclass(frozen=True, slots=True)
class Atom:
    """A leaf token, holding its exact source text.

    ``raw`` is what was in the file (including quotes, if any); ``text`` is the decoded
    value. Keeping ``raw`` is what stops numbers being reformatted on the way back out.
    """

    raw: str

    @property
    def is_quoted(self) -> bool:
        return len(self.raw) >= 2 and self.raw[0] == '"'

    @property
    def text(self) -> str:
        if not self.is_quoted:
            return self.raw
        body = self.raw[1:-1]
        out, i = [], 0
        while i < len(body):
            c = body[i]
            if c == "\\" and i + 1 < len(body):
                nxt = body[i + 1]
                out.append({"n": "\n", "t": "\t", "r": "\r"}.get(nxt, nxt))
                i += 2
            else:
                out.append(c)
                i += 1
        return "".join(out)

    def as_float(self) -> float:
        try:
            return float(self.text)
        except ValueError as exc:  # pragma: no cover - malformed board
            raise SExprError(f"expected a number, got {self.raw!r}") from exc

    def as_int(self) -> int:
        return int(self.as_float())

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Atom({self.raw})"


Node = list  # list[Atom | Node]; a plain list so callers can splice freely
Item = Union[Atom, Node]


# --------------------------------------------------------------------------- construction

def sym(name: str) -> Atom:
    """A bare symbol atom, e.g. ``segment`` or ``F.Cu`` written unquoted."""
    return Atom(name)


def quoted(value: str) -> Atom:
    """A quoted-string atom, escaping as KiCad does."""
    body = value.replace("\\", "\\\\").replace('"', '\\"')
    return Atom(f'"{body}"')


def num(value: float, places: int = 6) -> Atom:
    """A numeric atom formatted the way KiCad writes millimetres.

    Trailing zeros are stripped, and ``-0`` is normalised to ``0``, so that emitting a value
    twice always produces the same text.
    """
    if value == int(value) and abs(value) < 1e15:
        return Atom(str(int(value)))
    text = f"{value:.{places}f}".rstrip("0").rstrip(".")
    return Atom(text if text not in ("-0", "") else "0")


# --------------------------------------------------------------------------- parsing

_DELIM = set("() \t\r\n")


def _tokenize(text: str) -> Iterator[str]:
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            yield c
            i += 1
        elif c == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    break
                j += 1
            if j >= n:
                raise SExprError(f"unterminated string starting at offset {i}")
            yield text[i:j + 1]
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in _DELIM:
                j += 1
            yield text[i:j]
            i = j


def parse(text: str) -> Node:
    """Parse a complete s-expression document and return its single root node."""
    stack: list[Node] = []
    root: Node | None = None

    for tok in _tokenize(text):
        if tok == "(":
            node: Node = []
            if stack:
                stack[-1].append(node)
            stack.append(node)
        elif tok == ")":
            if not stack:
                raise SExprError("unbalanced ')'")
            node = stack.pop()
            if not stack:
                if root is not None:
                    raise SExprError("multiple root expressions")
                root = node
        else:
            if not stack:
                raise SExprError(f"atom {tok!r} outside any expression")
            stack[-1].append(Atom(tok))

    if stack:
        raise SExprError("unbalanced '(' -- unexpected end of input")
    if root is None:
        raise SExprError("empty document")
    return root


# --------------------------------------------------------------------------- serialising

def _is_flat(node: Node) -> bool:
    return all(isinstance(child, Atom) for child in node)


def _write(node: Node, out: list[str], depth: int) -> None:
    pad = "\t" * depth
    if _is_flat(node):
        out.append(pad + "(" + " ".join(a.raw for a in node) + ")")
        return

    lead: list[str] = []
    idx = 0
    while idx < len(node) and isinstance(node[idx], Atom):
        lead.append(node[idx].raw)
        idx += 1
    out.append(pad + "(" + " ".join(lead))
    for child in node[idx:]:
        if isinstance(child, Atom):
            out.append("\t" * (depth + 1) + child.raw)
        else:
            _write(child, out, depth + 1)
    out.append(pad + ")")


def dumps(node: Node) -> str:
    """Serialise a token tree back to text KiCad will load."""
    out: list[str] = []
    _write(node, out, 0)
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------- navigation

def head(node: Item) -> str:
    """The node's leading symbol, or ``""`` for an atom or empty node."""
    if isinstance(node, Atom) or not node:
        return ""
    first = node[0]
    return first.text if isinstance(first, Atom) else ""


def find_all(node: Node, name: str) -> list[Node]:
    """Every direct child node whose head is ``name``."""
    return [c for c in node if isinstance(c, list) and head(c) == name]


def find(node: Node, name: str) -> Node | None:
    """The first direct child node whose head is ``name``, or ``None``."""
    for child in node:
        if isinstance(child, list) and head(child) == name:
            return child
    return None


def atoms(node: Node) -> list[Atom]:
    """The node's atom children, excluding the leading symbol."""
    return [c for c in node[1:] if isinstance(c, Atom)]


def floats(node: Node | None, count: int | None = None) -> list[float]:
    """Numeric payload of a node such as ``(at 12.5 30 90)``."""
    if node is None:
        return []
    vals = [a.as_float() for a in atoms(node)]
    return vals if count is None else vals[:count]


def walk(node: Node) -> Iterator[Node]:
    """Depth-first iteration over every sub-node, including ``node`` itself."""
    yield node
    for child in node:
        if isinstance(child, list):
            yield from walk(child)


def remove_children(node: Node, names: Sequence[str]) -> int:
    """Delete direct children whose head is in ``names``; returns how many were removed."""
    wanted = set(names)
    keep = [c for c in node if not (isinstance(c, list) and head(c) in wanted)]
    removed = len(node) - len(keep)
    node[:] = keep
    return removed
