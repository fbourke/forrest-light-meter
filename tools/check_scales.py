#!/usr/bin/env python3
"""Validate the marked shutter/aperture scales in main/exposure.c.

Each table entry pairs an exact stop value (used for arithmetic) with a dial
engraving (used for display). This checks that the exact values really are
evenly spaced powers of two, that snapping can never be worse than half a
step, and that no engraving strays implausibly far from its exact value --
which is how a mistyped label shows up.
"""
import math
import re
import sys

SRC = "main/exposure.c"

# name, stop size, worst engraving rounding the convention actually uses
TABLES = [
    ("k_shutter_full",   1.0,     0.10),
    ("k_shutter_half",   0.5,     0.18),  # 1/10 is engraved on the 1/11.3 slot
    ("k_shutter_third",  1 / 3,   0.10),
    ("k_aperture_full",  1.0,     0.10),
    ("k_aperture_half",  0.5,     0.10),
    ("k_aperture_third", 1 / 3,   0.15),  # f/1.2 is engraved on the 1.26 stop
]


def parse(src, name):
    body = re.search(r"%s\[\] = \{(.*?)\n\};" % re.escape(name), src, re.S).group(1)
    return [(float(v.strip().rstrip("f")), lab)
            for v, lab in re.findall(r'\{([^,]+),\s*"([^"]+)"\}', body)]


def engraved_value(label):
    if label.endswith("s"):
        return float(label[:-1])
    if label.startswith("1/"):
        return 1.0 / float(label[2:])
    return float(label)


def main():
    src = open(SRC).read()
    failures = []

    for name, step, label_tol in TABLES:
        table = parse(src, name)
        # One stop is a factor of 2 in time but only sqrt(2) in f-number.
        k = 2 if "aperture" in name else 1

        gaps = [abs(k * math.log2(b[0] / a[0]) - step)
                for a, b in zip(table, table[1:])]
        if max(gaps) > 1e-3:
            failures.append("%s: uneven spacing, off by %.4f stop" % (name, max(gaps)))

        if any(b[0] <= a[0] for a, b in zip(table, table[1:])):
            failures.append("%s: not monotonic" % name)

        lo, hi = table[0][0], table[-1][0]
        snap = max(min(abs(k * math.log2(x / v)) for v, _ in table)
                   for x in (lo * (hi / lo) ** (i / 999) for i in range(1000)))
        if snap > step / 2 + 1e-3:
            failures.append("%s: snap error %.4f exceeds half a step" % (name, snap))

        rounding, worst = max((k * abs(math.log2(engraved_value(l) / v)), l)
                              for v, l in table)
        if rounding > label_tol:
            failures.append('%s: engraving "%s" is %.3f stop from its exact value'
                            % (name, worst, rounding))

        print("%-18s n=%-3d spacing %.1e  snap %.4f/%.4f  label %.3f (%s)"
              % (name, len(table), max(gaps), snap, step / 2, rounding, worst))

    print()
    for f in failures:
        print("FAIL", f)
    print("%d table(s) checked, %d problem(s)" % (len(TABLES), len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
