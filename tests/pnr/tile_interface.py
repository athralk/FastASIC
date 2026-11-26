"""Exercise the TT template CLI without a checkout of the shuttle repository."""
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory


with TemporaryDirectory() as tmp:
    work = Path(tmp)
    netlist = work / "buffer.v"
    netlist.write_text("module buffer(input a, output y); sg13cmos5l_buf_1 u (.A(a), .X(y)); endmodule\n")
    template = work / "tile.def"
    template.write_text("""VERSION 5.8 ;
UNITS DISTANCE MICRONS 1000 ;
DIEAREA ( 0 0 ) ( 100800 60480 ) ;
PINS 2 ;
- y + NET y + DIRECTION OUTPUT + USE SIGNAL + PORT
  + LAYER Metal4 ( -150 -500 ) ( 150 500 ) + PLACED ( 18720 59980 ) N ;
- a + NET a + DIRECTION INPUT + USE SIGNAL + PORT
  + LAYER Metal4 ( -150 -500 ) ( 150 500 ) + PLACED ( 23040 59980 ) N ;
END PINS
SPECIALNETS 2 ;
- VGND + USE GROUND ;
- VPWR + USE POWER ;
END SPECIALNETS
END DESIGN
""")
    routed = work / "routed.def"
    cmd = [sys.argv[1], "-tt-template", str(template), "-droute", "-o", str(routed), str(netlist)]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    result = routed.read_text()
    assert "DIEAREA ( 0 0 ) ( 100800 60480 )" in result
    pins = result.partition("\nPINS 4 ;")[2].partition("END PINS")[0]
    for name, x in (("a", 23040), ("y", 18720)):
        pin = re.search(rf" - {name} \+ NET {name} [^;]*", pins)[0]
        assert f"+ FIXED ( {x} 59980 ) N" in pin
        assert "+ LAYER Metal4 ( -150 -500 ) ( 150 500 )" in pin
    special = result.partition("SPECIALNETS 2 ;")[2].partition("END SPECIALNETS")[0]
    assert "TopMetal" not in special and "TopVia" not in special
    for name, cell_pin in (("VPWR", "VDD"), ("VGND", "VSS")):
        assert f"- {name} ( * {cell_pin} )" in special
        pin = re.search(rf" - {name} \+ NET {name} [^;]*", pins)[0]
        assert "+ PORT + LAYER Metal4" in pin
    template.write_text(template.read_text().replace("- a + NET a", "- unknown + NET a"))
    rejected = subprocess.run(cmd, capture_output=True, text=True)
    assert rejected.returncode and "unexpected pin unknown" in rejected.stderr
print("tile interface verified")
