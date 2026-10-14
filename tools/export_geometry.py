import sys
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
HOST = REPO / "host"
DEFAULT_CAD = REPO / "cad" / "arm" / "Arm.FCStd"

UNITS = {
    "App::PropertyLength": "mm",
    "App::PropertyDistance": "mm",
    "App::PropertyAngle": "deg",
}


def format_value(prop):
    value = prop[0]
    if value.tag == "Float":
        text = f"{float(value.get('value')):.6f}".rstrip("0").rstrip(".")
    elif value.tag == "Bool":
        text = value.get("value")
    else:
        text = value.get("value", "")
    unit = UNITS.get(prop.get("type"))
    return f"{text} {unit}" if unit else text


def read_varsets(cad_path):
    with zipfile.ZipFile(cad_path) as z:
        root = ET.fromstring(z.read("Document.xml"))

    varset_names = [o.get("name") for o in root.find("Objects") if o.get("type") == "App::VarSet"]
    data = {o.get("name"): o for o in root.find("ObjectData")}

    for name in varset_names:
        props = data[name].find("Properties").findall("Property")
        label = next((p[0].get("value") for p in props if p.get("name") == "Label"), name)
        expressions = {
            e.get("path"): e.get("expression")
            for p in props if p.get("name") == "ExpressionEngine"
            for e in p.iter("Expression")
        }
        variables = [p for p in props if p.get("group") is not None]
        yield label, variables, expressions


def main():
    cad_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CAD
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else HOST / (cad_path.stem + "_varsets.txt")

    lines = []
    for label, variables, expressions in read_varsets(cad_path):
        for prop in variables:
            name = prop.get("name")
            line = f"{label}.{name} = {format_value(prop)}"
            if name in expressions:
                line += f"  (= {expressions[name]})"
            if prop.get("doc"):
                line += f"  # {prop.get('doc')}"
            lines.append(line)

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {len(lines)} variables to {out_path}")


if __name__ == "__main__":
    main()
