# Warehouse Simulation

## Overview
This Python application simulates a warehouse environment where Autonomous Guided Vehicles (AGVs) transport materials between different locations. The simulation includes:
- **Draggable Objects:** Raw materials, tables, conveyors, AGVs, processed materials, final products, trash, and people.
- **AGV Automation:** Three AGVs perform tasks such as picking up raw materials, placing them on tables, and transferring processed materials to the final conveyor.
- **Dynamic Behavior:** Raw materials turn into processed materials after a delay.
- **Simulation Parameters:** Configurable settings like AGV speed, conveyor speed, and the number of materials.
- **Statistics Display:** Continuous tracking of material counts.
- **Save/Load World:** Ability to save and reload the warehouse setup.

## Installation
### Requirements
- Python 3.x
- PySimpleGUI

### Setup
1. Install dependencies using pip:
   ```bash
   pip install PySimpleGUI
   ```
2. Run the simulation:
   ```bash
   python warehouse_simulation.py
   ```

## Usage
1. **Adding Objects:** Click buttons to add raw materials and tables.
2. **Dragging Objects:** Move objects around the canvas.
3. **Saving & Loading:** Use the 'Save' and 'Load' buttons to store and restore setups.
4. **Starting Simulation:** Click 'Start Simulation' to run AGV automation.
5. **Tracking:** Monitor materials and AGVs in real time.

## Features
- **AGV 1:** Picks up raw material (brown box) from the raw area and places it on a table.
- **Processing Delay:** Raw materials turn red after processing.
- **AGV 2:** Moves processed materials to the final conveyor.
- **AGV 3:** Moves additional raw materials to a second table.
- **Dynamic AGV Movement:** AGVs navigate between locations.

## Future Enhancements
- Improved AGV pathfinding.
- Conveyor animations.
- Additional object interactions.

## License
This project is open-source under the MIT License.

## Author
Developed with AI assistance to demonstrate warehouse logistics in Python.

