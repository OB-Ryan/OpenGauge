# OpenGauge PCB Files

This folder contains both the editable EasyEDA source and the manufacturing outputs needed to order boards.

## Contents
- `OpenGauge.json`: EasyEDA source file (edit this file for schematic/PCB changes)
- `OpenGauge_Gerber/`: Gerber and drill files for PCB fabrication

## Ordering a PCB
1. Zip the full `OpenGauge_Gerber/` folder.
2. Upload the zip to your PCB manufacturer (for example, JLCPCB, PCBWay, or OSH Park).
3. Verify board outline and drill preview before placing the order.

## Notes
- Keep all files inside `OpenGauge_Gerber/` together.
- Do not delete drill files (`.DRL`) or outline/layer files (`.GKO`, `.GTL`, `.GBL`, etc.).
- Regenerate and replace the Gerber folder any time the design in `OpenGauge.json` changes.
