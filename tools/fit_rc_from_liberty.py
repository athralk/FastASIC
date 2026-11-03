#!/usr/bin/env python3
"""Fit the RC delay model to a Liberty cell's NLDM delay tables.

For every timing arc, the table row at the fastest input slew (closest to the ideal
step input the RC model assumes) is fitted by least squares with
    delay = intrinsic + R_eff * C_load
and the worst error of that line over every table entry is reported per input slew,
which shows how much the RC model loses by ignoring input slew.

These are the numbers in src/cell_library.cpp.

usage: fit_rc_from_liberty.py <file.lib> <cell> [<cell> ...]
  e.g. fit_rc_from_liberty.py sg13cmos5l_stdcell_typ_1p20V_25C.lib \\
           sg13cmos5l_inv_1 sg13cmos5l_nand2_1
"""
import re
import sys


def group_body(text, start):
    """Text between the braces of the group whose header starts at `start`."""
    i = text.index("{", start)
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[i + 1:j]
    raise ValueError("unbalanced braces")


def top_level(text):
    """`text` with every nested group removed, so attribute lookups only see this level."""
    while True:
        stripped = re.sub(r"\{[^{}]*\}", "", text)
        if stripped == text:
            return text
        text = stripped


def groups(text, kind):
    """(name, body) of each top-level `kind (name) { ... }` group in `text`."""
    for m in re.finditer(r"\b" + kind + r'\s*\(\s*"?([^")]*)"?\s*\)\s*\{', text):
        before = text[:m.start()]
        if before.count("{") == before.count("}"):
            yield m.group(1), group_body(text, m.start())


def attribute(text, name):
    m = re.search(r"\b" + name + r'\s*:\s*"?([^";]*)"?\s*;', top_level(text))
    return m.group(1).strip() if m else None


def numbers(text):
    return [float(x) for x in re.findall(r"[-+0-9.eE]+", text)]


def table(body):
    index_1 = numbers(re.search(r"index_1\s*\(([^)]*)\)", body).group(1))   # input slew (ns)
    index_2 = numbers(re.search(r"index_2\s*\(([^)]*)\)", body).group(1))   # output load (pF)
    values = numbers(re.search(r"values\s*\((.*?)\)\s*;", body, re.S).group(1))
    rows = [values[r * len(index_2):(r + 1) * len(index_2)] for r in range(len(index_1))]
    return index_1, index_2, rows


def line_fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return my - slope * mx, slope


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    text = open(sys.argv[1]).read()
    for cell in sys.argv[2:]:
        m = re.search(r'\bcell\s*\(\s*"?' + re.escape(cell) + r'"?\s*\)', text)
        if not m:
            sys.exit(f"cell {cell} not found")
        body = group_body(text, m.start())
        print(f"{cell}  area {attribute(body, 'area')} um^2")
        for pin, pin_body in groups(body, "pin"):
            if attribute(pin_body, "direction") == "input":
                print(f"  input pin {pin}: {float(attribute(pin_body, 'capacitance')) * 1000:.3f} fF")
                continue
            print(f"  output pin {pin}: max_capacitance {float(attribute(pin_body, 'max_capacitance')) * 1000:.2f} fF")
            for _, arc in groups(pin_body, "timing"):
                related = attribute(arc, "related_pin")
                for kind in ("cell_rise", "cell_fall"):
                    for _, tbody in groups(arc, kind):
                        slews, loads, rows = table(tbody)
                        intercept, slope = line_fit(loads, rows[0])   # ns and ns/pF = kOhm
                        errors = [max(abs(intercept + slope * c - d) for c, d in zip(loads, row)) for row in rows]
                        print(f"    {related}->{pin} {kind:<9}  intrinsic {intercept * 1000:7.3f} ps"
                              f"  R_eff {slope:.4f} kOhm  C_par {intercept / slope * 1000:.2f} fF")
                        print("        worst error vs table: " +
                              ", ".join(f"{e * 1000:.0f} ps @ {s * 1000:.0f} ps slew" for s, e in zip(slews, errors)))


if __name__ == "__main__":
    main()
