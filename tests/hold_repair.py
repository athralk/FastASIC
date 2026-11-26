"""Check default mapping against the two corner reports on an 8-bit shift register."""
import subprocess
import sys
import tempfile
from pathlib import Path

compiler, timer = (Path(arg).resolve() for arg in sys.argv[1:])
rtl = Path(__file__).parent / "seq/shift_register.v"


def invoke(*args):
    subprocess.run(args, check=True, stdout=subprocess.DEVNULL)


def slacks(work, cells, corner, check):
    report = work / "endpoints"
    invoke(timer, "-q", "-corner", corner, "-sdc", work / "test.sdc", "-endpoints", report, cells)
    rows = (line.split() for line in report.read_text().splitlines()[1:])
    return {(row[0], row[3]): float(row[-1]) for row in rows if row[1] == check}


with tempfile.TemporaryDirectory() as scratch:
    work = Path(scratch)
    base, repaired = work / "base.v", work / "repaired.v"
    invoke(compiler, "-q", "--map", "delay", "--no-buffer", "-o", base, rtl)
    for case, period, uncertainty in (("safe", 10, 0), ("repairable", 10, 0.25), ("setup-limited", 0.75, 0.25)):
        (work / "test.sdc").write_text(
            f"create_clock -name clk -period {period} [get_ports clk]\n"
            f"set_clock_uncertainty -hold {uncertainty} [get_clocks clk]\n"
            "set_propagated_clock [get_clocks clk]\nset_input_delay 1 -clock clk [get_ports d]\n")
        invoke(compiler, "-q", "--map", "delay", "--sdc", work / "test.sdc", "-o", repaired, rtl)
        original = slacks(work, base, "fast", "hold")
        final = slacks(work, repaired, "fast", "hold")
        setup = slacks(work, base, "slow", "setup")
        final_setup = slacks(work, repaired, "slow", "setup")
        assert original.keys() == final.keys() == setup.keys() == final_setup.keys()
        assert all(final_setup[k] >= min(0, old) for k, old in setup.items())
        assert (min(original.values()) < 0) == (case != "safe")
        if case == "repairable":
            assert min(final.values()) >= 0 and repaired.read_text() != base.read_text()
        else:
            assert repaired.read_text() == base.read_text(), case
            if case == "setup-limited":
                assert all(setup[k] > 0 for k in original if original[k] < 0)
print("fast hold and slow setup checks passed")
