"""Regression checks for routed bus names, RC loading, derating, clock phase and LVS."""
import argparse
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SDC = """create_clock -name clk -period [expr {1e+1}] [get_ports clk]
set_input_delay [expr {1e-1 - 0.1}] -clock clk [get_ports d]
set_output_delay 0 -clock clk [get_ports q]
"""


def run(args, *, stdin=None, expected=0):
    p = subprocess.run(args, text=True, input=stdin, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != expected:
        raise RuntimeError(f"{args} exited {p.returncode}\n{p.stdout}")
    return p.stdout


def value(pattern, output):
    m = re.search(pattern, output, re.MULTILINE)
    if not m:
        raise ValueError(f"missing {pattern} in\n{output}")
    return float(m[1])


def close(actual, expected, tol=0.02):
    if not math.isclose(actual, expected, abs_tol=tol):
        raise ValueError(f"expected {expected} ± {tol}, measured {actual}")


def verify(w, args):
    def sta(script):
        lib = Path(args.lib).resolve()
        image = os.environ.get("LIBRELANE_IMAGE", "ghcr.io/librelane/librelane:3.1.0.dev4")
        tcl = "read_liberty /lib.lib\n" + script
        output = run(["docker", "run", "--rm", "-i", "-v", f"{lib}:/lib.lib:ro", "-v", f"{w}:/w:ro",
                      "-v", f"{HERE}:/cases:ro", image, "sta", "-no_init", "-exit", "/dev/stdin"], stdin=tcl)
        if re.search(r"^(Error|Warning):", output, re.MULTILINE):
            raise ValueError(output)
        return output

    def route(netlist, name):
        run([ROOT / "fastpnr", "-q", "-droute", "-o", w / f"{name}.def", "-spef", w / f"{name}.spef", netlist])

    (w / "clock.sdc").write_text(SDC)
    route(HERE / "bus_rc.v", "bus")
    spef = (w / "bus.spef").read_text()
    for n in ["a[0]", "a[1]", "y[0]", "y[1]", "stage[0]", "stage[1]", r"literal\[3\]"]:
        assert f"*D_NET {n} " in spef, n
    assert r"*I i\[0\]_buf:A" in spef
    if args.signoff:
        report = sta("read_verilog /cases/bus_rc.v\nlink_design bus_rc\nread_spef /w/bus.spef\n"
                     "report_parasitic_annotation -report_unannotated\nreport_net -digits 6 {a[0]}\n")
        assert "Found 0 unannotated drivers." in report and "Found 0 partially unannotated drivers." in report
        cap = value(r"Wire capacitance: ([\d.]+)", report)
        close(cap, value(r"^\*D_NET a\[0\] ([\d.]+)", spef), 0.00001)
        print(f"OpenSTA annotated all seven bus/scalar nets, a[0] wire capacitance {cap} pF")

    def lsta(net, *extra):
        return run([ROOT / "lsta", "-lib", args.lib, "-sdc", w / "clock.sdc", *extra, net])

    # Elmore/lumped-C and OpenSTA's default delay calculator are different models.
    # Check load accounting and derate ratios independently of their absolute wire delays.
    for fixture, expected_cap in [("rc_slew", 101.60), ("rc_ccap", 211.60), ("rc_inter", 61.60)]:
        for derated in (False, True):
            factors = "set_timing_derate -late 1.1\nset_timing_derate -early 0.9\n" if derated else ""
            (w / "clock.sdc").write_text(SDC + factors)
            src = HERE / f"{fixture}.spef"
            out = lsta(HERE / "rc_test.v", "-spef", src)
            cap = value(r"^\s*([\d.]+)\s+[\d.]+\s+[\d.]+\s+[\d.]+ .\s+b1/X", out)
            arrival = value(r"^\s*[\d.]+\s+[\d.]+\s+([\d.]+) .\s+r2/D", out)
            slew = value(r"^\s*([\d.]+)\s+[\d.]+\s+[\d.]+ .\s+r2/D", out)
            slack = value(r"^  setup\s+([\d.]+)", out)
            close(cap, expected_cap)
            if derated:
                close(arrival, base_arrival * 1.1)
                close(slew, base_slew)
            else:
                base_arrival, base_slew = arrival, slew
            msg = f"{fixture} derate={derated}: lsta cap {cap} fF, slew {slew} ps, setup slack {slack} ps"
            if args.signoff:
                out = sta(f"read_verilog /cases/rc_test.v\nlink_design rc_test\nread_spef /cases/{src.name}\n"
                          + SDC + factors + "report_checks -path_delay max -to r2/D -fields {cap slew} -digits 6\n")
                cap_ref = 1000 * value(r"^\s*([\d.]+)\s+[\d.]+\s+[\d.]+\s+[\d.]+ .\s+b1/X", out)
                arrival_ref = 1000 * value(r"^\s*[\d.]+\s+[\d.]+\s+([\d.]+) .\s+r2/D", out)
                slew_ref = 1000 * value(r"^\s*([\d.]+)\s+[\d.]+\s+[\d.]+ .\s+r2/D", out)
                slack_ref = 1000 * value(r"^\s*([\d.]+)\s+slack \(", out)
                close(cap_ref, cap)
                if derated:
                    close(arrival_ref, ref_base * 1.1)
                else:
                    ref_base = arrival_ref
                msg += f"; OpenSTA cap {cap_ref:.2f} fF, slew {slew_ref:.2f} ps, setup slack {slack_ref:.2f} ps"
            print(msg)

    # The load-dependent part of an external driving cell is derated as net delay.
    drive = SDC + "set_driving_cell -lib_cell sg13cmos5l_buf_1 [get_ports d]\n"
    for factor in (1.0, 1.5):
        constraint = drive + f"set_timing_derate -net_delay -late {factor}\n"
        (w / "clock.sdc").write_text(constraint)
        out = lsta(HERE / "rc_test.v", "-paths", "4")
        d = value(r"^\s*[\d.]+\s+[\d.]+\s+([\d.]+) .\s+r1/D", out)
        if args.signoff:
            report = sta("read_verilog /cases/rc_test.v\nlink_design rc_test\n" + constraint +
                         "report_checks -path_delay max -to r1/D -digits 6 -fields {cap slew}\n")
            ref = 1000 * value(r"^\s*[\d.]+\s+[\d.]+\s+([\d.]+) .\s+r1/D", report)
            close(d, ref)
        if factor == 1:
            nominal = d
        else:
            close(d, factor * nominal)

    run([args.bin, "-q", "--map", "area", "-o", w / "half.v", HERE / "half_cycle.v"])
    for duty in (0.5, 0.3):
        clock = SDC.replace("[get_ports clk]", f"-waveform {{0 {10*duty}}} [get_ports clk]", 1)
        (w / "clock.sdc").write_text(clock)
        out = lsta(w / "half.v")
        close(value(r"skew ([\d.]+) ps", out), 0)
        period = value(r"Minimum period ([\d.]+)", out)
        if duty == 0.5:
            close(period, 529.17)
        if args.signoff:
            scaled = clock.replace("[expr {1e+1}]", str(period/1000)).replace(f"{{0 {10*duty}}}", f"{{0 {period*duty/1000}}}")
            out = sta("read_verilog /w/half.v\nlink_design half_cycle\n" + scaled + "report_worst_slack -max -digits 6\n")
            close(1000 * value(r"worst slack max ([-\d.]+)", out), 0)
        print(f"clock duty {duty:.0%}: minimum period {period} ps, ideal skew zero")

    if args.signoff:
        (w / "feed.v").write_text("module feed(input a, output y, hi, lo); assign y=a; assign hi=1'b1; assign lo=1'b0; endmodule\n")
        run([args.bin, "-q", "--map", "area", "-o", w / "cells.v", w / "feed.v"])
        route(w / "cells.v", "feed")
        lvs = ["bash", ROOT / "tools/lvs_klayout.sh"]
        out = run([*lvs, w / "feed.def"])
        assert "opens 0, shorts 0" in out and "LVS: netlists match" in out, out
        layout = (w / "feed.def").read_text()
        layout, count = re.subn(r"(\( PIN a \) \( PIN y \)[^;]+?)\s+\+ ROUTED [^;]+;", r"\1 ;", layout)
        assert count == 1
        (w / "open.def").write_text(layout)
        out = run([*lvs, w / "open.def"], expected=1)
        assert "OPEN " in out, out
        print("KLayout matched tie-high, tie-low and feedthrough; removing feedthrough metal failed the pin check")
        (w / "wire.v").write_text("module metal_only(input a, output y); assign y=a; endmodule\n")
        run([args.bin, "-q", "--map", "area", "-o", w / "wire_cells.v", w / "wire.v"])
        route(w / "wire_cells.v", "wire")
        out = run([*lvs, w / "wire.def"])
        assert "opens 0, shorts 0" in out and "LVS: netlists match" in out, out
        print("KLayout matched the device-free top and verified all four signal and supply pins")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", default=str(ROOT / "FastASIC"))
    parser.add_argument("--lib", required=True)
    parser.add_argument("--signoff", action="store_true", default=os.environ.get("FASTASIC_SIGNOFF") == "1",
                        help="also check with OpenSTA and KLayout in Docker")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        verify(Path(temp), args)
