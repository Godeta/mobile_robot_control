import FreeSimpleGUI as sg
import json
import time
import threading

# temporary code in development It currently only allows to show some basic shapes like a static red, green or gray rectangle. 
# Save the objects and load 

# Constants
CANVAS_SIZE = (600, 400)
RAW_COLOR = "brown"
PROCESSED_COLOR = "red"
TABLE_COLOR = "gray"
FINAL_COLOR = "blue"
AGV_COLOR = "green"
CONVEYOR_COLOR = "yellow"

class WarehouseObject:
    """Base class for warehouse objects."""
    def __init__(self, object_type, x, y, color):
        self.type = object_type
        self.x = x
        self.y = y
        self.color = color

    def to_dict(self):
        """Convert object data to dictionary for saving."""
        return {"type": self.type, "x": self.x, "y": self.y, "color": self.color}

    @staticmethod
    def from_dict(data):
        """Create object from saved data."""
        return WarehouseObject(data["type"], data["x"], data["y"], data["color"])

#temporary code just to show a green rectangle
class AGV:
    """Autonomous Guided Vehicle (AGV) class."""
    def __init__(self, x, y, task):
        self.x = x
        self.y = y
        self.task = task
        self.carrying = None

    def move_to(self, target_x, target_y):
        """Move AGV to target coordinates."""
        self.x, self.y = target_x, target_y

    def pick_up(self, obj):
        """Pick up an object."""
        self.carrying = obj
        obj.x, obj.y = self.x, self.y

    def drop_off(self, target_x, target_y):
        """Drop off an object."""
        if self.carrying:
            self.carrying.x, self.carrying.y = target_x, target_y
            self.carrying = None

class Warehouse:
    """Manages warehouse objects and AGVs."""
    def __init__(self):
        self.objects = []
        self.agvs = [
            AGV(50, 50, "raw_to_table"),
            AGV(100, 50, "table_to_final"),
            AGV(150, 50, "raw_to_table")
        ]
        self.running = True

    def add_object(self, obj):
        """Add a new object."""
        self.objects.append(obj)

    def save_world(self, filename):
        """Save world configuration to file."""
        with open(filename, 'w') as f:
            json.dump([obj.to_dict() for obj in self.objects], f)

    def load_world(self, filename):
        """Load world configuration from file."""
        with open(filename, 'r') as f:
            self.objects = [WarehouseObject.from_dict(data) for data in json.load(f)]

    def run_simulation(self):
        """Run warehouse automation simulation."""
        while self.running:
            for agv in self.agvs:
                if agv.task == "raw_to_table":
                    raw_materials = [obj for obj in self.objects if obj.type == "RawMaterial"]
                    if raw_materials:
                        agv.pick_up(raw_materials[0])
                        agv.move_to(200, 200)  # Move to table
                        agv.drop_off(200, 200)
                        time.sleep(1)
                        raw_materials[0].color = PROCESSED_COLOR  # Processing step
                elif agv.task == "table_to_final":
                    processed = [obj for obj in self.objects if obj.color == PROCESSED_COLOR]
                    if processed:
                        agv.pick_up(processed[0])
                        agv.move_to(500, 200)  # Move to final conveyor
                        agv.drop_off(500, 200)
            time.sleep(2)

class GUIManager:
    """Manages GUI interactions."""
    def __init__(self):
        self.warehouse = Warehouse()
        self.selected_type = "RawMaterial"  # Default selection
        
        layout = [
            [sg.Button("Add Raw Material"), sg.Button("Add Table"), sg.Button("Save"), sg.Button("Load"), sg.Button("Start Simulation")],
            [sg.Graph(CANVAS_SIZE, (0, 0), CANVAS_SIZE, key="-CANVAS-", enable_events=True, drag_submits=True)]
        ]
        
        self.window = sg.Window("Warehouse Simulation", layout, finalize=True)
        self.canvas = self.window["-CANVAS-"]
        self.dragging = None

    def draw_objects(self):
        """Redraw all objects on the canvas."""
        self.canvas.erase()
        for obj in self.warehouse.objects:
            self.canvas.draw_rectangle((obj.x, obj.y), (obj.x+40, obj.y+40), fill_color=obj.color, line_color="black")
        for agv in self.warehouse.agvs:
            self.canvas.draw_rectangle((agv.x, agv.y), (agv.x+30, agv.y+30), fill_color=AGV_COLOR, line_color="black")

    def run(self):
        """Main event loop."""
        while True:
            event, values = self.window.read()
            if event == sg.WINDOW_CLOSED:
                self.warehouse.running = False
                break
            elif event == "Add Raw Material":
                self.warehouse.add_object(WarehouseObject("RawMaterial", 50, 50, RAW_COLOR))
            elif event == "Add Table":
                self.warehouse.add_object(WarehouseObject("Table", 100, 100, TABLE_COLOR))
            elif event == "Save":
                self.warehouse.save_world("warehouse.json")
            elif event == "Load":
                self.warehouse.load_world("warehouse.json")
            elif event == "Start Simulation":
                threading.Thread(target=self.warehouse.run_simulation, daemon=True).start()
            
            self.draw_objects()

        self.window.close()

if __name__ == "__main__":
    gui = GUIManager()
    gui.run()