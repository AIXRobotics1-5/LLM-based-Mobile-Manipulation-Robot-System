"""Carter AMR + Doosan E0509 Digital Twin — Command-Driven

Layout (matches factory floor plan):
    ┌──────────────────────────────┐
    │        Camera (top)          │
    │                              │
    │  ┌─────────┐    ┌──────┐    │
    │  │  입고    │    │ 출고  │    │
    │  │(Inbound)│    │(Out) │    │
    │  └─────────┘    └──────┘    │
    │                              │
    │       ◯ Robot (AMR+E0509)    │
    │                              │
    │  ┌────┬────┬────┐           │
    │  │ C  │ B  │ A  │ Work zones│
    │  └────┴────┴────┘           │
    └──────────────────────────────┘

Command-driven workflow:
  1. System starts in IDLE — AMR at home, waiting for commands
  2. "inbound" → AMR goes to inbound area. Vision-detected objects are
     spawned in Isaac Sim at their robot-base-frame coordinates.
  3. "work A" / "work B" / "work C" → AMR navigates to the zone,
     then enters arm-sync mode (digital twin with real robot).
  4. "done" → finishes arm work, returns to IDLE.
  5. "outbound" → AMR goes to outbound area.
  6. "home" → AMR returns to home position.

Commands are received via:
  - Keyboard (stdin) in standalone mode
  - ROS2 topic /digital_twin/command (std_msgs/String) when ROS2 is available
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import json
import math
import os
import queue
import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import carb.events
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

# ZMQ for ROS2 bridge communication (no rclpy needed in Isaac Sim)
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[DigitalTwin] zmq not available — install with: pip install pyzmq")

ROS2_AVAILABLE = ZMQ_AVAILABLE


# ---------------------------------------------------------------------------
# Enums & Data
# ---------------------------------------------------------------------------
class RobotMode(Enum):
    IDLE = auto()
    AMR_MOVING = auto()
    ARM_WORKING = auto()


@dataclass
class WorkZone:
    name: str
    position: tuple  # (x, y, z)
    color: tuple     # (r, g, b)
    size: tuple = (1.5, 1.5, 0.02)


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
LAYOUT = {
    # ===================================================================
    # Real table dimensions (robot arm center = origin)
    #   X: left 23cm, right 34cm  → total width 57cm
    #   Y: arm=0, A/B/C=-24cm, conveyor(입고)=-44cm, camera=-90cm
    # Scale: 1:1 (cm → m)
    # Arm faces -Y direction (toward A/B/C, conveyor)
    # 출고는 tcp 하드코딩이므로 AMR 상판에 포함하지 않음
    # ===================================================================

    # --- Factory floor zones (where AMR delivers) ---
    "zones": [
        WorkZone("A", (3.0, -3.0, 0.0), (1.0, 0.8, 0.7)),   # pink
        WorkZone("B", (0.0, -3.0, 0.0), (0.7, 1.0, 0.7)),    # green
        WorkZone("C", (-3.0, -3.0, 0.0), (0.7, 0.8, 1.0)),   # blue
    ],
    # Conveyor belt (입고) — fixed station in factory
    # Conveyor top = AMR lift plate top = 0.231m
    # Conveyor is thin table-top that AMR slides under (하단부가 밑으로 들어감)
    # Table legs are thinner than table top, so AMR chassis fits under
    "conveyor":  {"position": (0.0, 5.0, 0.0), "size": (1.0, 0.5, 0.231)},
    # AMR docking: AMR slides so lift plate edge meets conveyor edge
    # IW Hub lift plate right edge: X=+0.255 from AMR origin
    # AMR chassis wider part goes under conveyor
    # Conveyor back edge Y = 5.0 - 0.5 = 4.5
    # Lift plate Y extent: ±0.33 → sector at Y=+0.20 reaches Y=AMR+0.20
    # For sectors to touch conveyor: AMR_Y + 0.20 = 4.5 → AMR_Y = 4.3
    "inbound_dock": (0.0, 4.3, 0.0),
    # Outbound area (출고) — in factory
    "outbound":  {"position": (5.0, -3.0, 0.0), "size": (1.5, 1.5, 0.8)},
    # Camera — overhead
    "camera":    {"position": (0.0, 0.0, 8.0), "target": (0.0, -0.45, 0.0)},
    # AMR home
    "robot_home": (0.0, 1.0, 0.0),

    # --- AMR on-board layout (relative to AMR origin) ---
    # IW Hub lift plate: X=-0.768~+0.255, Y=-0.329~+0.330
    # Plate right edge X=0.255, so sectors align to right edge
    #
    #   ┌─────────┬──────────┐  X=0.255 (plate edge)
    #   │         │    C     │
    #   │  로봇팔 ├──────────┤
    #   │    ◯    │    B     │
    #   │         ├──────────┤
    #   │         │    A     │
    #   └─────────┴──────────┘
    #   X=-0.05   X=0.075~0.255

    # Sector offsets from curobo_planner place coordinates (mm→m):
    #   A center: (340, -135, 150) → X=0.34, Y=-0.135
    #   B center: (340,   30, 150) → X=0.34, Y= 0.03
    #   C center: (340,  200, 150) → X=0.34, Y= 0.20
    "amr_sectors": {
        "A": {"offset": (0.34, -0.135, 0.0), "color": (0.9, 0.15, 0.15)},   # red
        "B": {"offset": (0.34,  0.03,  0.0), "color": (0.15, 0.8, 0.15)},   # green
        "C": {"offset": (0.34,  0.20,  0.0), "color": (0.95, 0.85, 0.1)},   # yellow
    },
    "amr_sector_size": (0.06, 0.08),  # ~12cm x 16cm per sector
    # Robot arm on plate, left of sectors
    "arm_offset_on_amr": (-0.05, 0.0, 0.0),
}

E0509_JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]


# ---------------------------------------------------------------------------
# Block colors and their properties
# ---------------------------------------------------------------------------
BLOCK_COLORS = {
    "red":    {"rgb": (0.9, 0.15, 0.15), "zone": "A", "korean": "빨간"},
    "green":  {"rgb": (0.15, 0.8, 0.15), "zone": "B", "korean": "초록"},
    "yellow": {"rgb": (0.95, 0.85, 0.1), "zone": "C", "korean": "노란"},
}

# Position offsets within a zone (relative to zone center)
POSITION_OFFSETS = {
    "back":   (0.0, 0.4, 0.0),   # far from robot
    "front":  (0.0, -0.4, 0.0),  # near robot
    "center": (0.0, 0.0, 0.0),
}


# ---------------------------------------------------------------------------
# Scenario Manager — matches keywords, executes task lists
# ---------------------------------------------------------------------------
class ScenarioManager:
    """Loads scenarios.json and matches user input to predefined scenarios."""

    def __init__(self, json_path: str):
        self._scenarios = []
        self._pending_followup = None  # for "ask" type scenarios
        if os.path.exists(json_path):
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._scenarios = data.get("scenarios", [])
            print(f"[Scenario] Loaded {len(self._scenarios)} scenarios")
        else:
            print(f"[Scenario] WARNING: {json_path} not found")

    def match(self, user_input: str) -> Optional[dict]:
        """Try to match user input against scenario keywords.
        Returns the scenario output dict if matched, None otherwise.
        """
        # Check if this is a followup answer (1 or 2)
        if self._pending_followup and user_input.strip() in ("1", "2"):
            followup = self._pending_followup.get(user_input.strip())
            self._pending_followup = None
            return followup

        for scenario in self._scenarios:
            keywords = scenario.get("keywords", [])
            if all(kw in user_input for kw in keywords):
                output = scenario["output"]
                if output["action"] == "ask":
                    # Store followup for next input
                    self._pending_followup = output.get("followup", {})
                return output
        return None

    @property
    def has_pending_followup(self):
        return self._pending_followup is not None


# ---------------------------------------------------------------------------
# Block Manager — tracks R/G/Y blocks on AMR table + sim
# ---------------------------------------------------------------------------
class BlockManager:
    """Manages wooden blocks in simulation.

    Supports two coordinate modes:
      - "conveyor": spawn on the fixed conveyor belt (world coords)
      - "amr": spawn on AMR sector A/B/C (AMR-relative, needs amr_pos)
      - "factory": spawn at factory floor zone (world coords)

    Workflow:
      1. inbound → spawn_on_conveyor(colors)
      2. after real arm picks to AMR → spawn_on_amr_sector(color, sector)
      3. AMR moves to factory zone → blocks on AMR are cleared
      4. camera re-scans → spawn at outbound TCP or remove
    """

    BLOCK_SIZE = (0.04, 0.04, 0.04)  # 4cm wooden cube

    def __init__(self, stage, spawner):
        self._stage = stage
        self._spawner = spawner
        self._blocks: List[dict] = []

    # ----- Conveyor (입고) -----
    def spawn_on_conveyor(self, colors: List[str]):
        """Spawn blocks side by side on the conveyor belt.

        colors: e.g. ["red", "green", "yellow"]
        """
        conv_pos = LAYOUT["conveyor"]["position"]
        conv_size = LAYOUT["conveyor"]["size"]
        # z = conveyor top surface + half block
        z = conv_size[2] + self.BLOCK_SIZE[2] / 2

        n = len(colors)
        spacing = min(conv_size[0] / max(n, 1), 0.15)
        start_x = conv_pos[0] - spacing * (n - 1) / 2

        spawned = []
        for i, color in enumerate(colors):
            if color not in BLOCK_COLORS:
                print(f"[Block] Unknown color: {color}")
                continue
            x = start_x + i * spacing
            y = conv_pos[1]
            rgb = BLOCK_COLORS[color]["rgb"]
            prim_path = self._spawner.spawn(x, y, z, f"{color}_block")
            block = {
                "color": color,
                "zone": "conveyor",
                "prim_path": prim_path,
                "world_pos": (x, y, z),
            }
            self._blocks.append(block)
            spawned.append(block)
            print(f"[Block] {color} on conveyor ({x:.2f}, {y:.2f}, {z:.2f})")
        return spawned

    # ----- AMR sector (A/B/C on AMR top) -----
    def spawn_on_amr_sector(self, color: str, sector: str, amr_pos):
        """Spawn a block on AMR sector A/B/C at current AMR world position.

        Called after real arm completes pick & place, camera confirms.
        amr_pos: current AMR world position (x, y, z).
        """
        if color not in BLOCK_COLORS:
            print(f"[Block] Unknown color: {color}")
            return None
        if sector not in LAYOUT["amr_sectors"]:
            print(f"[Block] Unknown sector: {sector}")
            return None

        # Remove existing block of same color on conveyor
        self._remove_blocks(color=color, zone="conveyor")

        offset = LAYOUT["amr_sectors"][sector]["offset"]
        x = amr_pos[0] + offset[0]
        y = amr_pos[1] + offset[1]
        z = amr_pos[2] + 0.15 + self.BLOCK_SIZE[2] / 2  # AMR origin + top(0.15) + half block

        prim_path = self._spawner.spawn(x, y, z, f"{color}_block")
        block = {
            "color": color,
            "zone": f"amr_{sector}",
            "prim_path": prim_path,
            "world_pos": (x, y, z),
        }
        self._blocks.append(block)
        print(f"[Block] {color} on AMR sector {sector} ({x:.2f}, {y:.2f}, {z:.2f})")
        return block

    # ----- Re-spawn on AMR after camera scan -----
    def rescan_amr(self, sector_colors: Dict[str, str], amr_pos):
        """Re-spawn blocks on AMR based on camera scan results.

        sector_colors: e.g. {"A": "red", "B": "green", "C": "yellow"}
        Clears old AMR blocks and creates new ones.
        """
        # Clear all amr blocks
        self._remove_blocks(zone_prefix="amr_")

        for sector, color in sector_colors.items():
            if color:
                self.spawn_on_amr_sector(color, sector, amr_pos)

    # ----- Outbound (출고 완료 후 제거) -----
    def remove_from_amr_sector(self, sector: str):
        """Remove block from AMR sector after outbound pick & place."""
        zone_name = f"amr_{sector}"
        removed = self._remove_blocks(zone=zone_name)
        if removed:
            print(f"[Block] Cleared AMR sector {sector} (outbound complete)")
        else:
            print(f"[Block] No block in AMR sector {sector}")

    # ----- Factory floor spawn (for scenario compatibility) -----
    def spawn_block(self, color: str, zone: str, position: str = "center"):
        """Spawn block at factory floor zone (legacy)."""
        if color not in BLOCK_COLORS:
            print(f"[Block] Unknown color: {color}")
            return None

        zone_positions = {z.name: z.position for z in LAYOUT["zones"]}
        if zone == "D":
            zone_pos = LAYOUT["outbound"]["position"]
        elif zone in zone_positions:
            zone_pos = zone_positions[zone]
        else:
            print(f"[Block] Unknown zone: {zone}")
            return None

        offset = POSITION_OFFSETS.get(position, POSITION_OFFSETS["center"])
        x = zone_pos[0] + offset[0]
        y = zone_pos[1] + offset[1]
        z = 0.8 + self.BLOCK_SIZE[2] / 2  # table height

        prim_path = self._spawner.spawn(x, y, z, f"{color}_block")
        block = {
            "color": color,
            "zone": zone,
            "prim_path": prim_path,
            "world_pos": (x, y, z),
        }
        self._blocks.append(block)
        print(f"[Block] {color} at factory zone {zone} ({x:.2f}, {y:.2f}, {z:.2f})")
        return block

    def move_block(self, color: str, to_zone: str, to_position: str = "center"):
        """Move existing block or spawn new one."""
        block = next((b for b in self._blocks if b["color"] == color), None)
        if not block:
            return self.spawn_block(color, to_zone, to_position)

        zone_positions = {z.name: z.position for z in LAYOUT["zones"]}
        if to_zone == "D":
            zone_pos = LAYOUT["outbound"]["position"]
        elif to_zone in zone_positions:
            zone_pos = zone_positions[to_zone]
        else:
            print(f"[Block] Unknown zone: {to_zone}")
            return None

        offset = POSITION_OFFSETS.get(to_position, POSITION_OFFSETS["center"])
        x = zone_pos[0] + offset[0]
        y = zone_pos[1] + offset[1]
        z = 0.8 + self.BLOCK_SIZE[2] / 2

        prim = self._stage.GetPrimAtPath(block["prim_path"])
        if prim:
            attr = prim.GetAttribute("xformOp:translate")
            if attr and attr.IsValid():
                attr.Set(Gf.Vec3d(x, y, z))

        block["zone"] = to_zone
        block["world_pos"] = (x, y, z)
        print(f"[Block] {color} moved to zone {to_zone} ({x:.2f}, {y:.2f}, {z:.2f})")
        return block

    # ----- Queries -----
    def get_blocks_in_zone(self, zone: str) -> List[dict]:
        return [b for b in self._blocks if b["zone"] == zone]

    def get_blocks_on_amr(self) -> List[dict]:
        return [b for b in self._blocks if b["zone"].startswith("amr_")]

    def get_all_blocks(self) -> List[dict]:
        return list(self._blocks)

    # ----- Internal -----
    def _remove_blocks(self, color: str = None, zone: str = None,
                       zone_prefix: str = None) -> int:
        """Remove blocks matching criteria. Returns count removed."""
        to_remove = []
        for b in self._blocks:
            if color and b["color"] != color:
                continue
            if zone and b["zone"] != zone:
                continue
            if zone_prefix and not b["zone"].startswith(zone_prefix):
                continue
            to_remove.append(b)

        for b in to_remove:
            prim = self._stage.GetPrimAtPath(b["prim_path"])
            if prim:
                self._stage.RemovePrim(b["prim_path"])
            self._blocks.remove(b)
        return len(to_remove)


# ---------------------------------------------------------------------------
# Keyboard Command Reader (stdin, non-blocking)
# ---------------------------------------------------------------------------
class KeyboardCommandReader:
    """Reads commands from stdin in a background thread."""

    def __init__(self, cmd_queue: queue.Queue):
        self._queue = cmd_queue
        self._running = False

    def start(self):
        self._running = True
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    def stop(self):
        self._running = False

    def _read_loop(self):
        print("\n" + "=" * 60)
        print("[DigitalTwin] Command-Driven Mode")
        print("  Navigation:")
        print("    inbound              — AMR → conveyor (입고)")
        print("    work A|B|C           — AMR → factory zone, arm sync")
        print("    done                 — finish arm work → IDLE")
        print("    home                 — AMR → home")
        print("  Inbound (입고):")
        print("    load r,g,y           — manual spawn on conveyor")
        print("    (auto: /dsr01/curobo/obstacles → real-time vision spawn)")
        print("    loaded A red         — block picked to AMR sector A")
        print("  Outbound (출고):")
        print("    unload A             — remove block from AMR sector A")
        print("  Block info:")
        print("    blocks               — show all blocks")
        print("    rescan A=red,B=green — re-spawn AMR blocks from camera")
        print("  Scenario (Korean):")
        print("    say <자연어 명령>     — 시나리오 매칭")
        print("  Other:")
        print("    status / quit")
        print("=" * 60 + "\n")

        while self._running:
            try:
                line = input("[cmd]> ").strip()
                if line:
                    self._queue.put(line)
            except EOFError:
                break


# ---------------------------------------------------------------------------
# ROS2 Bridge Node (commands + joint states + object poses)
# ---------------------------------------------------------------------------
class ROS2Bridge:
    """ZMQ-based bridge to receive ROS2 data from ros2_bridge_node.py.

    ros2_bridge_node.py (Python 3.10/ROS2) publishes to tcp://localhost:5557
    This class subscribes via ZMQ (Python 3.11/Isaac Sim) — no rclpy needed.

    Messages:
      topic "obstacles" → JSON: [{name, pos, dims}]
      topic "joints"    → JSON: {name: [...], position: [...]}
      topic "command"   → string command
    """

    def __init__(self, cmd_queue: queue.Queue):
        self._cmd_queue = cmd_queue
        self._ctx = None
        self._sub = None
        self._latest_joints = [0.0] * 6
        self._gripper_stroke = 0  # 0=open, 700=closed
        self._gripper_updated = False
        self._obstacles = []
        self._obstacles_updated = False
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        if not ZMQ_AVAILABLE:
            print("[DigitalTwin] ZMQ not available")
            return
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect("tcp://localhost:5557")
        self._sub.setsockopt(zmq.SUBSCRIBE, b"obstacles")
        self._sub.setsockopt(zmq.SUBSCRIBE, b"joints")
        self._sub.setsockopt(zmq.SUBSCRIBE, b"gripper")
        self._sub.setsockopt(zmq.SUBSCRIBE, b"command")
        self._sub.setsockopt(zmq.RCVTIMEO, 1)  # 1ms timeout (non-blocking)
        self._running = True
        print("[DigitalTwin] ZMQ bridge connected to tcp://localhost:5557")
        print("  Run ros2_bridge_node.py in another terminal for ROS2 data")

    def stop(self):
        self._running = False
        if self._sub:
            self._sub.close()
        if self._ctx:
            self._ctx.term()

    def poll(self):
        """Non-blocking receive of all pending ZMQ messages."""
        if not self._running or not self._sub:
            return
        while True:
            try:
                topic, data = self._sub.recv_multipart()
                topic = topic.decode()
                data = data.decode()

                if topic == "obstacles":
                    parsed = json.loads(data)
                    with self._lock:
                        self._obstacles = parsed
                        self._obstacles_updated = True

                elif topic == "joints":
                    parsed = json.loads(data)
                    names = parsed.get("name", [])
                    positions = parsed.get("position", [])
                    with self._lock:
                        for i, jname in enumerate(E0509_JOINT_NAMES):
                            if jname in names:
                                idx = names.index(jname)
                                if idx < len(positions):
                                    self._latest_joints[i] = positions[idx]

                elif topic == "gripper":
                    with self._lock:
                        self._gripper_stroke = int(data)
                        self._gripper_updated = True

                elif topic == "command":
                    self._cmd_queue.put(data.strip())

            except zmq.Again:
                break  # no more messages
            except Exception:
                break

    def get_joint_positions(self):
        with self._lock:
            return list(self._latest_joints)

    def get_gripper_stroke(self):
        """Returns (stroke, updated). stroke: 0=open, 700=closed."""
        with self._lock:
            updated = self._gripper_updated
            self._gripper_updated = False
            return self._gripper_stroke, updated

    def get_obstacles(self):
        with self._lock:
            updated = self._obstacles_updated
            self._obstacles_updated = False
            return list(self._obstacles), updated


# ---------------------------------------------------------------------------
# Object Spawner — spawns detected objects in Isaac Sim
# ---------------------------------------------------------------------------
class ObjectSpawner:
    """Manages spawning/removing objects in the Isaac Sim stage."""

    COLORS = {
        "pen":          (0.2, 0.3, 0.9),
        "box":          (0.8, 0.5, 0.2),
        "object":       (0.9, 0.9, 0.3),
        "red_block":    (0.9, 0.15, 0.15),
        "green_block":  (0.15, 0.8, 0.15),
        "yellow_block": (0.95, 0.85, 0.1),
        "default":      (0.7, 0.7, 0.7),
    }
    SIZES = {
        "pen":          (0.005, 0.005, 0.08),
        "box":          (0.03, 0.03, 0.03),
        "red_block":    (0.04, 0.04, 0.04),
        "green_block":  (0.04, 0.04, 0.04),
        "yellow_block": (0.04, 0.04, 0.04),
        "object":       (0.02, 0.02, 0.02),
        "default":      (0.02, 0.02, 0.02),
    }

    def __init__(self, stage):
        self._stage = stage
        self._objects = []  # list of prim paths
        self._counter = 0

    def spawn(self, x, y, z, label="object"):
        """Spawn an object at (x, y, z) in world frame."""
        self._counter += 1
        safe_label = label.replace(" ", "_").replace("/", "_")
        prim_path = f"/SpawnedObjects/{safe_label}_{self._counter}"

        # Choose shape: pen → cylinder, others → cube
        if "pen" in label.lower():
            shape = UsdGeom.Cylinder.Define(self._stage, prim_path)
            size = self.SIZES.get(label.lower(), self.SIZES["default"])
            shape.GetRadiusAttr().Set(size[0])
            shape.GetHeightAttr().Set(size[2] * 2)
        else:
            shape = UsdGeom.Cube.Define(self._stage, prim_path)
            size = self.SIZES.get(label.lower(), self.SIZES["default"])
            xformable = UsdGeom.Xformable(shape.GetPrim())
            if not shape.GetPrim().GetAttribute("xformOp:scale"):
                xformable.AddScaleOp()
            shape.GetPrim().GetAttribute("xformOp:scale").Set(
                Gf.Vec3d(*size)
            )

        # Position
        xformable = UsdGeom.Xformable(shape.GetPrim())
        if not shape.GetPrim().GetAttribute("xformOp:translate"):
            xformable.AddTranslateOp()
        shape.GetPrim().GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(x, y, z)
        )

        # Color
        color = self.COLORS.get(label.lower(), self.COLORS["default"])
        shape.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        # Physics
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(shape.GetPrim())

        self._objects.append(prim_path)
        print(f"[Spawner] {label} at ({x:.3f}, {y:.3f}, {z:.3f}) → {prim_path}")
        return prim_path

    def clear_all(self):
        """Remove all spawned objects."""
        for path in self._objects:
            prim = self._stage.GetPrimAtPath(path)
            if prim:
                self._stage.RemovePrim(path)
        count = len(self._objects)
        self._objects.clear()
        print(f"[Spawner] Cleared {count} objects")

    def count(self):
        return len(self._objects)


# ---------------------------------------------------------------------------
# Main Demo Class
# ---------------------------------------------------------------------------
class CarterE0509DigitalTwin:
    """Command-driven Carter AMR + E0509 digital twin."""

    # AMR model — IW Hub (flat-top logistics AMR)
    AMR_URL = "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd"
    _WS_SRC = os.path.expanduser("~/doosan_ws/src")
    E0509_USD = os.path.join(
        _WS_SRC, "doosan-robot2", "dsr_description2", "usd", "e0509.usd"
    )
    # E0509 + Gripper USD (Isaac Lab version, already exists)
    E0509_GRIPPER_USD = os.path.expanduser(
        "~/IsaacLab/e0509_gripper_isaaclab/e0509_gripper_isaaclab.usd"
    )

    ENV_PATH = "/Environment"
    FACTORY_ENV_FILE = "factory_layout.usd"
    ENV_TRANSLATE = (2.5, 5.0, 0.0)
    ENV_ROTATE_XYZ = (0.0, 0.0, -90.0)

    # IW Hub dimensions (from bbox measurement):
    #   Top Z = 0.15 from AMR origin
    #   Bottom Z = -0.081 (wheels)
    #   When placed on ground, AMR origin shifts up by 0.081
    #   → actual top surface Z from ground = 0.081 + 0.15 = 0.231
    #   Center X = -0.318 (asymmetric, long side is -X)
    AMR_TOP_Z_FROM_ORIGIN = 0.15      # relative to AMR prim origin
    AMR_BOTTOM_Z = -0.081             # wheel bottom
    AMR_CENTER_X = -0.318             # geometric center X
    MANIPULATOR_LOCAL_OFFSET = Gf.Vec3d(-0.05, 0.0, AMR_TOP_Z_FROM_ORIGIN)
    MANIPULATOR_LOCAL_ROTATE_XYZ = Gf.Vec3d(0.0, 0.0, 0.0)

    GOAL_TOLERANCE = 0.5
    QUIT_REQUESTED = False

    def __init__(self, enable_ros2=True):
        self._stage = None
        self._timeline = None
        self._timeline_sub = None
        self._stage_event_sub = None
        self._running = False

        # State
        self._mode = RobotMode.IDLE
        self._amr_root_prim = None
        self._carter_root_prim = None       # legacy alias
        self._carter_chassis_prim = None
        self._carter_target_prim = None
        self._manip_prim = None
        self._joint_prims = {}
        self._amr_sector_prims = {}

        # Navigation
        self._current_goal_name = None
        self._on_arrive_callback = None  # called when AMR arrives at goal

        # Arm sync
        self._arm_work_tick_count = 0
        self._physics_dt = 1.0 / 60.0
        self._vision_spawn_enabled = False  # only after inbound docking

        # Command queue (shared between keyboard + ROS2)
        self._cmd_queue = queue.Queue()

        # Subsystems
        self._enable_ros2 = enable_ros2 and ROS2_AVAILABLE
        self._ros2_bridge = ROS2Bridge(self._cmd_queue) if self._enable_ros2 else None
        self._kbd_reader = KeyboardCommandReader(self._cmd_queue)
        self._spawner = None  # initialized after stage creation
        self._block_mgr = None  # initialized after stage creation

        # Scenario manager
        scenario_json = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "scenarios.json"
        )
        self._scenario_mgr = ScenarioManager(scenario_json)

        # Task queue for sequential task execution
        self._task_queue: List[dict] = []

        # Waypoints
        self._waypoints = {}
        for zone in LAYOUT["zones"]:
            self._waypoints[zone.name] = zone.position
        self._waypoints["robot_home"] = LAYOUT["robot_home"]
        self._waypoints["inbound"] = LAYOUT["inbound_dock"]
        self._waypoints["outbound"] = LAYOUT["outbound"]["position"]

        # AMR direct-drive navigation state
        self._amr_nav_speed = 1.5  # m/s

    # -------------------------------------------------------------------
    # Public
    # -------------------------------------------------------------------
    def start(self):
        self._setup_stage()
        self._spawner = ObjectSpawner(self._stage)
        self._block_mgr = BlockManager(self._stage, self._spawner)
        self._set_position("robot_home")

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        self._timeline_sub = (
            self._timeline.get_timeline_event_stream()
            .create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED),
                self._on_timeline_tick,
            )
        )
        self._stage_event_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop_by_type(
                int(omni.usd.StageEventType.CLOSING), self._on_stage_closing
            )
        )

        if self._ros2_bridge:
            self._ros2_bridge.start()
        self._kbd_reader.start()

        self._mode = RobotMode.IDLE
        self._running = True
        print("[DigitalTwin] Ready — IDLE at home. Waiting for commands.")

    def stop(self):
        if self._timeline_sub:
            self._timeline_sub.unsubscribe()
            self._timeline_sub = None
        if self._stage_event_sub:
            self._stage_event_sub.unsubscribe()
            self._stage_event_sub = None
        if self._ros2_bridge:
            self._ros2_bridge.stop()
        self._kbd_reader.stop()
        self._running = False

    def is_running(self):
        return self._running and not self.QUIT_REQUESTED

    # -------------------------------------------------------------------
    # Stage setup (same as before)
    # -------------------------------------------------------------------
    def _setup_stage(self):
        create_new_stage()
        self._stage = omni.usd.get_context().get_stage()
        self._add_physics_scene()
        self._add_lighting()
        self._add_ground_plane()
        self._load_factory_environment()
        self._build_layout()
        self._load_amr_and_e0509()
        self._setup_camera()

    def _add_physics_scene(self):
        UsdPhysics.Scene.Define(self._stage, "/physicsScene")
        physx = PhysxSchema.PhysxSceneAPI.Apply(
            self._stage.GetPrimAtPath("/physicsScene")
        )
        physx.GetEnableCCDAttr().Set(True)
        physx.GetEnableGPUDynamicsAttr().Set(False)
        physx.GetBroadphaseTypeAttr().Set("MBP")

    def _load_factory_environment(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        env_usd = os.path.abspath(
            os.path.join(script_dir, "..", "asset", self.FACTORY_ENV_FILE)
        )
        if not os.path.exists(env_usd):
            print(f"[DigitalTwin] WARNING: factory layout not found: {env_usd}")
            return
        add_reference_to_stage(usd_path=env_usd, prim_path=self.ENV_PATH)
        env_prim = self._stage.GetPrimAtPath(self.ENV_PATH)
        env_xform = UsdGeom.Xformable(env_prim)
        if not env_prim.GetAttribute("xformOp:translate"):
            env_xform.AddTranslateOp()
        if not env_prim.GetAttribute("xformOp:rotateXYZ"):
            env_xform.AddRotateXYZOp()
        env_prim.GetAttribute("xformOp:translate").Set(self.ENV_TRANSLATE)
        env_prim.GetAttribute("xformOp:rotateXYZ").Set(self.ENV_ROTATE_XYZ)
        print(f"[DigitalTwin] Factory environment loaded")

    def _add_lighting(self):
        dome = UsdLux.DomeLight.Define(self._stage, "/Lighting/DomeLight")
        dome.GetIntensityAttr().Set(1000.0)
        dome.GetTextureFormatAttr().Set("latlong")

        sun = UsdLux.DistantLight.Define(self._stage, "/Lighting/SunLight")
        sun.GetIntensityAttr().Set(3000.0)
        sun.GetAngleAttr().Set(0.53)
        sun_xform = UsdGeom.Xformable(sun.GetPrim())
        if not sun.GetPrim().GetAttribute("xformOp:rotateXYZ"):
            sun_xform.AddRotateXYZOp()
        sun.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
            Gf.Vec3d(-45.0, 0.0, -30.0)
        )
        print("[DigitalTwin] Lighting added")

    def _add_ground_plane(self):
        from isaacsim.core.utils.prims import create_prim
        create_prim(prim_path="/GroundPlane", prim_type="Xform")
        ground_path = "/GroundPlane/CollisionPlane"
        UsdGeom.Mesh.Define(self._stage, ground_path)
        plane_prim = self._stage.GetPrimAtPath(ground_path)
        UsdPhysics.CollisionAPI.Apply(plane_prim)
        plane_mesh = UsdGeom.Mesh(plane_prim)
        half = 20.0
        plane_mesh.GetPointsAttr().Set([
            Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, -half, 0),
            Gf.Vec3f(half, half, 0), Gf.Vec3f(-half, half, 0),
        ])
        plane_mesh.GetFaceVertexCountsAttr().Set([4])
        plane_mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
        plane_mesh.GetNormalsAttr().Set([Gf.Vec3f(0, 0, 1)] * 4)

    def _build_layout(self):
        for zone in LAYOUT["zones"]:
            self._create_zone_marker(zone)
        self._create_table(
            "/Layout/Outbound", LAYOUT["outbound"]["position"],
            LAYOUT["outbound"]["size"], (0.6, 0.6, 0.65), "출고 (Outbound)",
        )

    def _create_zone_marker(self, zone: WorkZone):
        prim_path = f"/Layout/Zone_{zone.name}"
        cube = UsdGeom.Cube.Define(self._stage, prim_path)
        xf = UsdGeom.Xformable(cube.GetPrim())
        if not cube.GetPrim().GetAttribute("xformOp:translate"):
            xf.AddTranslateOp()
        if not cube.GetPrim().GetAttribute("xformOp:scale"):
            xf.AddScaleOp()
        cube.GetPrim().GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(zone.position[0], zone.position[1], zone.size[2] / 2)
        )
        cube.GetPrim().GetAttribute("xformOp:scale").Set(
            Gf.Vec3d(zone.size[0] / 2, zone.size[1] / 2, zone.size[2] / 2)
        )
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*zone.color)])
        print(f"[DigitalTwin] Zone {zone.name} at {zone.position}")

    def _create_table(self, prim_path, position, size, color, label=""):
        cube = UsdGeom.Cube.Define(self._stage, prim_path)
        xf = UsdGeom.Xformable(cube.GetPrim())
        if not cube.GetPrim().GetAttribute("xformOp:translate"):
            xf.AddTranslateOp()
        if not cube.GetPrim().GetAttribute("xformOp:scale"):
            xf.AddScaleOp()
        cube.GetPrim().GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(position[0], position[1], size[2] / 2)
        )
        cube.GetPrim().GetAttribute("xformOp:scale").Set(
            Gf.Vec3d(size[0] / 2, size[1] / 2, size[2] / 2)
        )
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if label:
            print(f"[DigitalTwin] {label} at {position}")

    def _load_amr_and_e0509(self):
        """Load IW Hub AMR and E0509 as SEPARATE scene-root prims.

        E0509 is NOT nested under AMR to avoid physics conflicts.
        It follows the AMR kinematically via _sync_arm_to_amr().

        AMR on-board sectors (A/B/C) are also separate prims that follow.
        """
        assets_root = get_assets_root_path()
        amr_path = "/NavWorld/AMR"

        # --- Load IW Hub AMR ---
        add_reference_to_stage(
            usd_path=assets_root + self.AMR_URL, prim_path=amr_path
        )
        self._amr_root_prim = self._stage.GetPrimAtPath(amr_path)
        if not self._amr_root_prim.GetAttribute("xformOp:translate"):
            UsdGeom.Xformable(self._amr_root_prim).AddTranslateOp()
        if not self._amr_root_prim.GetAttribute("xformOp:rotateXYZ"):
            UsdGeom.Xformable(self._amr_root_prim).AddRotateXYZOp()

        # IW Hub doesn't have nav target — we control position directly
        self._amr_has_nav = False

        # Try to find chassis/base link for IW Hub
        for candidate in ["base_link", "chassis_link", "body"]:
            p = self._stage.GetPrimAtPath(f"{amr_path}/{candidate}")
            if p and p.IsValid():
                self._carter_chassis_prim = p
                break
        else:
            # Fallback: use root prim as chassis
            self._carter_chassis_prim = self._amr_root_prim

        # Legacy compat (navigation uses these names internally)
        self._carter_root_prim = self._amr_root_prim
        self._carter_target_prim = None  # IW Hub has no nav target

        # Disable AMR physics — we drive it kinematically (direct position set)
        self._disable_all_physics(amr_path)
        print(f"[DigitalTwin] IW Hub AMR loaded (kinematic mode)")

        # --- Create on-board sector border markers (follow AMR) ---
        # Each sector: colored outline (4 thin bars forming rectangle)
        sec_size = LAYOUT.get("amr_sector_size", (0.09, 0.09))
        bar_t = 0.004  # bar thickness (half-extent)
        self._amr_sector_prims = {}  # stores the Xform parent per sector
        for sec_name, sec_info in LAYOUT["amr_sectors"].items():
            parent_path = f"/NavWorld/AMR_Sector_{sec_name}"
            parent_xf = UsdGeom.Xform.Define(self._stage, parent_path)
            prim = parent_xf.GetPrim()
            if not prim.GetAttribute("xformOp:translate"):
                UsdGeom.Xformable(prim).AddTranslateOp()
            color = Gf.Vec3f(*sec_info["color"])
            hw, hd = sec_size  # half-width, half-depth

            # 4 bars: top, bottom, left, right
            bars = {
                "top":    ((0, hd, 0),     (hw, bar_t, bar_t)),
                "bottom": ((0, -hd, 0),    (hw, bar_t, bar_t)),
                "left":   ((-hw, 0, 0),    (bar_t, hd, bar_t)),
                "right":  ((hw, 0, 0),      (bar_t, hd, bar_t)),
            }
            for bar_name, (bpos, bscale) in bars.items():
                bar_path = f"{parent_path}/{bar_name}"
                bar = UsdGeom.Cube.Define(self._stage, bar_path)
                bxf = UsdGeom.Xformable(bar.GetPrim())
                if not bar.GetPrim().GetAttribute("xformOp:translate"):
                    bxf.AddTranslateOp()
                if not bar.GetPrim().GetAttribute("xformOp:scale"):
                    bxf.AddScaleOp()
                bar.GetPrim().GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(*bpos)
                )
                bar.GetPrim().GetAttribute("xformOp:scale").Set(
                    Gf.Vec3d(*bscale)
                )
                bar.GetDisplayColorAttr().Set([color])

            self._amr_sector_prims[sec_name] = prim
            print(f"[DigitalTwin] AMR sector {sec_name} border ({sec_info['color']})")

        # --- Load E0509 at scene root ---
        manip_path = "/NavWorld/E0509_Arm"
        gripper_usd = os.path.abspath(self.E0509_GRIPPER_USD)
        e0509_usd = os.path.abspath(self.E0509_USD)
        if os.path.exists(gripper_usd):
            usd_to_load = gripper_usd
            print(f"[DigitalTwin] Loading E0509+Gripper: {gripper_usd}")
        else:
            usd_to_load = e0509_usd
            print(f"[DigitalTwin] Loading E0509: {e0509_usd}")

        add_reference_to_stage(usd_path=usd_to_load, prim_path=manip_path)
        self._manip_prim = self._stage.GetPrimAtPath(manip_path)

        manip_xform = UsdGeom.Xformable(self._manip_prim)
        if not self._manip_prim.GetAttribute("xformOp:translate"):
            manip_xform.AddTranslateOp()
        if not self._manip_prim.GetAttribute("xformOp:rotateXYZ"):
            manip_xform.AddRotateXYZOp()

        # Keep E0509 physics/articulation alive (it's at scene root, no conflicts)
        # Only disable gravity so it doesn't fall
        self._disable_gravity_on_arm(manip_path)
        self._discover_joint_drive_prims(manip_path)

        # --- Create conveyor belt at inbound station ---
        self._create_conveyor()

        # Initial sync
        self._sync_arm_to_amr()
        print(f"[DigitalTwin] E0509 follows AMR kinematically")

    def _create_conveyor(self):
        """Create conveyor as table with legs — AMR chassis fits underneath.

        Structure:
          ┌──────────────┐  ← table top (thin, at AMR top height)
          │              │
          ║              ║  ← legs (thin, tall enough for AMR to slide under)
          ║              ║
        """
        conv = LAYOUT["conveyor"]
        pos = conv["position"]
        size = conv["size"]  # (half_w, half_d, top_z)
        top_z = size[2]      # = 0.231 (matches AMR top)
        top_thickness = 0.02
        leg_width = 0.03

        # --- Table top (thin slab at top_z) ---
        top_path = "/Layout/Conveyor/top"
        top_cube = UsdGeom.Cube.Define(self._stage, top_path)
        xf = UsdGeom.Xformable(top_cube.GetPrim())
        if not top_cube.GetPrim().GetAttribute("xformOp:translate"):
            xf.AddTranslateOp()
        if not top_cube.GetPrim().GetAttribute("xformOp:scale"):
            xf.AddScaleOp()
        top_cube.GetPrim().GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(pos[0], pos[1], top_z - top_thickness / 2)
        )
        top_cube.GetPrim().GetAttribute("xformOp:scale").Set(
            Gf.Vec3d(size[0], size[1], top_thickness / 2)
        )
        top_cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.35)])

        # --- 4 legs (corners) ---
        leg_height = top_z - top_thickness
        leg_positions = [
            (pos[0] - size[0] + leg_width, pos[1] - size[1] + leg_width),
            (pos[0] + size[0] - leg_width, pos[1] - size[1] + leg_width),
            (pos[0] - size[0] + leg_width, pos[1] + size[1] - leg_width),
            (pos[0] + size[0] - leg_width, pos[1] + size[1] - leg_width),
        ]
        for i, (lx, ly) in enumerate(leg_positions):
            leg_path = f"/Layout/Conveyor/leg_{i}"
            leg = UsdGeom.Cube.Define(self._stage, leg_path)
            lxf = UsdGeom.Xformable(leg.GetPrim())
            if not leg.GetPrim().GetAttribute("xformOp:translate"):
                lxf.AddTranslateOp()
            if not leg.GetPrim().GetAttribute("xformOp:scale"):
                lxf.AddScaleOp()
            leg.GetPrim().GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(lx, ly, leg_height / 2)
            )
            leg.GetPrim().GetAttribute("xformOp:scale").Set(
                Gf.Vec3d(leg_width, leg_width, leg_height / 2)
            )
            leg.GetDisplayColorAttr().Set([Gf.Vec3f(0.25, 0.25, 0.28)])

        print(f"[DigitalTwin] Conveyor table at {pos} (top_z={top_z:.3f}, AMR fits under)")

    def _disable_all_physics(self, root_path):
        """Completely disable physics on all prims under root_path.

        Uses attribute overrides (works on referenced USD) and
        deactivates joint/collision sub-prims.
        """
        root_prim = self._stage.GetPrimAtPath(root_path)
        if not root_prim:
            return

        count = 0
        for prim in Usd.PrimRange(root_prim):
            # Disable rigid body
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.CreateAttribute(
                    "physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool
                ).Set(False)
                count += 1

            # Disable collisions
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.CreateAttribute(
                    "physics:collisionEnabled", Sdf.ValueTypeNames.Bool
                ).Set(False)

            # Deactivate collision-named prims
            name_lower = prim.GetName().lower()
            if "collision" in name_lower or "collider" in name_lower:
                prim.SetActive(False)

            # Deactivate physics joints
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)

            # Disable articulation root
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.CreateAttribute(
                    "physics:articulationEnabled", Sdf.ValueTypeNames.Bool
                ).Set(False)

        print(f"[DigitalTwin] Physics disabled on {count} prims under {root_path}")

    def _sync_arm_to_amr(self):
        """Update E0509 + on-board sectors to follow AMR position.

        Called every frame. Arm and sectors are at scene root level,
        so we read AMR's world transform and apply offsets.
        """
        if not self._carter_chassis_prim or not self._manip_prim:
            return

        # Get AMR world transform
        amr_xf = UsdGeom.Xformable(
            self._carter_chassis_prim
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        amr_pos = amr_xf.ExtractTranslation()

        # Extract yaw from AMR rotation
        amr_rot = amr_xf.ExtractRotation()
        axis = amr_rot.GetAxis()
        angle = amr_rot.GetAngle()
        yaw = angle if axis[2] > 0 else -angle
        yaw_rad = math.radians(yaw)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        # --- Sync robot arm ---
        arm_off = self.MANIPULATOR_LOCAL_OFFSET
        arm_world = Gf.Vec3d(
            amr_pos[0] + arm_off[0] * cos_y - arm_off[1] * sin_y,
            amr_pos[1] + arm_off[0] * sin_y + arm_off[1] * cos_y,
            amr_pos[2] + arm_off[2],
        )
        self._manip_prim.GetAttribute("xformOp:translate").Set(arm_world)
        arm_rot_base = self.MANIPULATOR_LOCAL_ROTATE_XYZ
        self._manip_prim.GetAttribute("xformOp:rotateXYZ").Set(
            Gf.Vec3d(arm_rot_base[0], arm_rot_base[1], arm_rot_base[2] + yaw)
        )

        # --- Sync on-board sector markers (position + rotation) ---
        amr_top_z = amr_pos[2] + self.AMR_TOP_Z_FROM_ORIGIN
        for sec_name, sec_info in LAYOUT["amr_sectors"].items():
            if sec_name in self._amr_sector_prims:
                off = sec_info["offset"]
                sec_world = Gf.Vec3d(
                    amr_pos[0] + off[0] * cos_y - off[1] * sin_y,
                    amr_pos[1] + off[0] * sin_y + off[1] * cos_y,
                    amr_top_z,
                )
                prim = self._amr_sector_prims[sec_name]
                prim.GetAttribute("xformOp:translate").Set(sec_world)
                # Match AMR yaw rotation
                rot_attr = prim.GetAttribute("xformOp:rotateXYZ")
                if not rot_attr or not rot_attr.IsValid():
                    UsdGeom.Xformable(prim).AddRotateXYZOp()
                prim.GetAttribute("xformOp:rotateXYZ").Set(
                    Gf.Vec3d(0.0, 0.0, yaw)
                )

    def _disable_gravity_on_arm(self, manip_path):
        """Disable gravity on E0509 so it doesn't fall, but keep articulation."""
        manip_prim = self._stage.GetPrimAtPath(manip_path)
        if not manip_prim:
            return
        for prim in Usd.PrimRange(manip_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                # Disable gravity on each rigid body
                physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                physx_rb.GetDisableGravityAttr().Set(True)
        print(f"[DigitalTwin] Gravity disabled on E0509 (articulation preserved)")

    def _discover_joint_drive_prims(self, manip_path):
        """Find revolute joint prims for drive-target control.

        Instead of rotating links directly (breaks kinematic chain),
        we set drive targets on the physics joints.
        """
        manip_prim = self._stage.GetPrimAtPath(manip_path)
        if not manip_prim:
            return

        for prim in Usd.PrimRange(manip_prim):
            if prim.IsA(UsdPhysics.RevoluteJoint):
                name = prim.GetName()
                for expected in E0509_JOINT_NAMES:
                    if expected in name.lower() or expected == name:
                        self._joint_prims[expected] = prim
                        # Ensure drive API exists with high stiffness
                        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                        drive.GetStiffnessAttr().Set(1e6)
                        drive.GetDampingAttr().Set(1e4)
                        break

        print(f"[DigitalTwin] Joint drives: {list(self._joint_prims.keys())}")

    def _setup_camera(self):
        """Set up a follow camera that tracks the AMR."""
        cam = UsdGeom.Camera.Define(self._stage, "/Layout/FollowCamera")
        self._follow_cam_prim = cam.GetPrim()
        xf = UsdGeom.Xformable(self._follow_cam_prim)
        if not self._follow_cam_prim.GetAttribute("xformOp:translate"):
            xf.AddTranslateOp()
        if not self._follow_cam_prim.GetAttribute("xformOp:rotateXYZ"):
            xf.AddRotateXYZOp()
        cam.GetFocalLengthAttr().Set(15.0)
        # Initial position
        self._follow_cam_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(1.5, -1.0, 1.2)
        )
        self._follow_cam_prim.GetAttribute("xformOp:rotateXYZ").Set(
            Gf.Vec3d(-30.0, 0.0, 120.0)
        )
        print(f"[DigitalTwin] Follow camera (tracks AMR)")

    def _update_follow_camera(self):
        """Update follow camera to track AMR position."""
        if not hasattr(self, "_follow_cam_prim") or not self._follow_cam_prim:
            return
        amr_pos = self._get_amr_position()
        # Camera offset from AMR: behind-right, elevated
        cam_x = amr_pos[0] + 1.2
        cam_y = amr_pos[1] - 0.8
        cam_z = amr_pos[2] + 1.0
        self._follow_cam_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(cam_x, cam_y, cam_z)
        )
        # Keep looking at AMR
        dx = amr_pos[0] - cam_x
        dy = amr_pos[1] - cam_y
        yaw = math.degrees(math.atan2(dy, dx))
        self._follow_cam_prim.GetAttribute("xformOp:rotateXYZ").Set(
            Gf.Vec3d(-30.0, 0.0, yaw)
        )

    # -------------------------------------------------------------------
    # Navigation helpers
    # -------------------------------------------------------------------
    def _set_position(self, waypoint_name):
        pos = self._waypoints[waypoint_name]
        self._set_amr_position(pos)

    def _navigate_to(self, waypoint_name, on_arrive=None):
        """Start navigating to a waypoint. on_arrive is called when reached."""
        if waypoint_name not in self._waypoints:
            print(f"[DigitalTwin] Unknown waypoint: {waypoint_name}")
            return
        self._current_goal_name = waypoint_name
        self._on_arrive_callback = on_arrive
        self._mode = RobotMode.AMR_MOVING
        print(f"[DigitalTwin] Navigating to {waypoint_name}...")

    def _get_amr_position(self):
        """Get AMR world position."""
        attr = self._amr_root_prim.GetAttribute("xformOp:translate")
        if attr and attr.IsValid():
            return attr.Get()
        return Gf.Vec3d(0, 0, 0)

    def _set_amr_position(self, pos):
        """Directly set AMR world position. Adjusts Z so wheels touch ground."""
        ground_z = -self.AMR_BOTTOM_Z  # lift AMR so wheels sit on Z=0 ground
        self._amr_root_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(pos[0], pos[1], ground_z)
        )

    def _has_arrived(self):
        if not self._current_goal_name:
            return False
        goal = self._waypoints[self._current_goal_name]
        pos = self._get_amr_position()
        dist = math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)
        return dist <= self.GOAL_TOLERANCE

    # -------------------------------------------------------------------
    # Arm control
    # -------------------------------------------------------------------
    def _apply_joint_positions(self, positions):
        """Apply joint angles via physics drive targets.

        Uses DriveAPI to set target position on each revolute joint.
        positions: list of 6 joint angles in radians.
        """
        for i, jname in enumerate(E0509_JOINT_NAMES):
            if jname in self._joint_prims:
                prim = self._joint_prims[jname]
                angle_deg = math.degrees(positions[i])
                target_attr = prim.GetAttribute("drive:angular:physics:targetPosition")
                if target_attr and target_attr.IsValid():
                    target_attr.Set(angle_deg)
                else:
                    prim.CreateAttribute(
                        "drive:angular:physics:targetPosition",
                        Sdf.ValueTypeNames.Float
                    ).Set(angle_deg)

    def _sync_gripper(self, stroke):
        """Sync gripper visual state and attach/detach nearest vision object.

        stroke: 0=fully open, 700=fully closed.
        When closing (stroke > 350): find nearest vision object and attach to gripper.
        When opening (stroke < 350): detach object.
        """
        gripper_closed = stroke > 350

        if not hasattr(self, "_gripper_attached_obj"):
            self._gripper_attached_obj = None
            self._prev_gripper_closed = False

        # State transition: open → closed (grab)
        if gripper_closed and not self._prev_gripper_closed:
            # Find the nearest vision object to the gripper (link_6/tool0)
            tool_prim = None
            manip_prim = self._stage.GetPrimAtPath("/NavWorld/E0509_Arm")
            if manip_prim:
                for prim in Usd.PrimRange(manip_prim):
                    if "tool0" in prim.GetName() or "link_6" in prim.GetName():
                        tool_prim = prim
                        break

            if tool_prim and hasattr(self, "_vision_prims"):
                tool_xf = UsdGeom.Xformable(tool_prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                )
                tool_pos = tool_xf.ExtractTranslation()

                nearest_name = None
                nearest_dist = float("inf")
                for name, path in self._vision_prims.items():
                    obj_prim = self._stage.GetPrimAtPath(path)
                    if obj_prim:
                        obj_attr = obj_prim.GetAttribute("xformOp:translate")
                        if obj_attr:
                            obj_pos = obj_attr.Get()
                            dist = math.sqrt(
                                (tool_pos[0] - obj_pos[0]) ** 2
                                + (tool_pos[1] - obj_pos[1]) ** 2
                                + (tool_pos[2] - obj_pos[2]) ** 2
                            )
                            if dist < nearest_dist:
                                nearest_dist = dist
                                nearest_name = name

                if nearest_name and nearest_dist < 0.15:  # within 15cm
                    self._gripper_attached_obj = nearest_name
                    print(f"[Gripper] GRABBED: {nearest_name} (dist={nearest_dist:.3f}m)")

        # State transition: closed → open (release)
        elif not gripper_closed and self._prev_gripper_closed:
            if self._gripper_attached_obj:
                print(f"[Gripper] RELEASED: {self._gripper_attached_obj}")
                self._gripper_attached_obj = None

        # If object is attached, move it with the gripper
        if self._gripper_attached_obj and hasattr(self, "_vision_prims"):
            if self._gripper_attached_obj in self._vision_prims:
                path = self._vision_prims[self._gripper_attached_obj]
                obj_prim = self._stage.GetPrimAtPath(path)
                if obj_prim:
                    # Get tool0 world position
                    manip_prim = self._stage.GetPrimAtPath("/NavWorld/E0509_Arm")
                    if manip_prim:
                        for prim in Usd.PrimRange(manip_prim):
                            if "tool0" in prim.GetName():
                                tool_xf = UsdGeom.Xformable(
                                    prim
                                ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                                tool_pos = tool_xf.ExtractTranslation()
                                obj_prim.GetAttribute("xformOp:translate").Set(
                                    Gf.Vec3d(tool_pos[0], tool_pos[1], tool_pos[2])
                                )
                                break

        self._prev_gripper_closed = gripper_closed

    # -------------------------------------------------------------------
    # Command processing
    # -------------------------------------------------------------------
    def _process_commands(self):
        """Drain the command queue and execute commands."""
        while not self._cmd_queue.empty():
            try:
                raw = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            self._execute_command(raw)

    def _execute_command(self, raw: str):
        parts = raw.split()
        cmd = parts[0].lower()

        if cmd == "quit":
            print("[DigitalTwin] Quit requested")
            self.QUIT_REQUESTED = True
            return

        if cmd == "home":
            self._navigate_to("robot_home", on_arrive=self._on_arrive_idle)
            return

        if cmd == "inbound":
            def on_arrive():
                self._vision_spawn_enabled = True
                print("[DigitalTwin] Docked at conveyor — vision spawn ENABLED")
                self._mode = RobotMode.IDLE

            self._navigate_to("inbound", on_arrive=on_arrive)
            return

        if cmd == "work" and len(parts) >= 2:
            zone = parts[1].upper()
            if zone not in ("A", "B", "C"):
                print(f"[DigitalTwin] Invalid zone: {zone}. Use A, B, or C.")
                return

            def on_arrive_work():
                self._mode = RobotMode.ARM_WORKING
                self._arm_work_tick_count = 0
                self._vision_spawn_enabled = True
                print(f"[DigitalTwin] ARM_WORKING at zone {zone} — vision spawn ENABLED")
                print("  Syncing with real robot. Type 'done' to finish.")

            self._navigate_to(zone, on_arrive=on_arrive_work)
            return

        if cmd == "done":
            if self._mode == RobotMode.ARM_WORKING:
                print("[DigitalTwin] Arm work done → IDLE")
                self._mode = RobotMode.IDLE
            else:
                print("[DigitalTwin] Not in ARM_WORKING mode")
            return

        if cmd == "spawn" and len(parts) >= 2:
            # Parse "spawn x,y,z [label]" or "spawn x y z [label]"
            try:
                coords_str = parts[1]
                if "," in coords_str:
                    xyz = coords_str.split(",")
                    x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
                    label = parts[2] if len(parts) > 2 else "object"
                else:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    label = parts[4] if len(parts) > 4 else "object"
                self._spawner.spawn(x, y, z, label)
            except (ValueError, IndexError):
                print("[DigitalTwin] Usage: spawn x,y,z [label]")
            return

        if cmd == "clear_objects":
            self._spawner.clear_all()
            return

        # --- Inbound: spawn on conveyor ---
        if cmd == "load":
            # load r,g,y  or  load red green yellow
            if len(parts) >= 2:
                color_input = " ".join(parts[1:]).replace(",", " ").split()
                color_map = {"r": "red", "g": "green", "y": "yellow",
                             "red": "red", "green": "green", "yellow": "yellow"}
                colors = [color_map.get(c.lower(), c.lower()) for c in color_input]
                self._block_mgr.spawn_on_conveyor(colors)
            else:
                print("[DigitalTwin] Usage: load r,g,y")
            return

        # --- After arm picks from conveyor to AMR sector ---
        if cmd == "loaded" and len(parts) >= 3:
            # loaded A red  → just log, vision handles the spawn
            sector = parts[1].upper()
            color = parts[2].lower()
            print(f"[DigitalTwin] Noted: {color} placed in sector {sector} (vision tracks position)")
            return

        # --- Outbound: remove from AMR sector ---
        if cmd == "unload" and len(parts) >= 2:
            # unload A  → block picked from AMR sector A to outbound TCP
            sector = parts[1].upper()
            self._block_mgr.remove_from_amr_sector(sector)
            return

        # --- Re-scan AMR from camera ---
        if cmd == "rescan":
            # Vision handles object tracking automatically
            print("[DigitalTwin] Rescan: vision objects update automatically")
            return

        if cmd == "blocks":
            blocks = self._block_mgr.get_all_blocks()
            if not blocks:
                print("[Block] No blocks")
            else:
                for b in blocks:
                    print(
                        f"  {b['color']:8s} @ {b['zone']:12s} "
                        f"({b['world_pos'][0]:.2f}, {b['world_pos'][1]:.2f}, "
                        f"{b['world_pos'][2]:.2f})"
                    )
            return

        # --- Scenario matching (Korean natural language) ---
        if cmd == "say":
            user_text = raw[4:].strip()  # everything after "say "
            if not user_text:
                print("[Scenario] Usage: say <자연어 명령>")
                return
            self._handle_scenario(user_text)
            return

        # Also try scenario matching for followup answers (1, 2)
        if self._scenario_mgr.has_pending_followup and cmd in ("1", "2"):
            self._handle_scenario(raw.strip())
            return

        if cmd == "status":
            pos = self._get_amr_position()
            print(f"[DigitalTwin] Mode: {self._mode.name}")
            print(f"  AMR position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"  Spawned objects: {self._spawner.count()}")
            blocks = self._block_mgr.get_all_blocks()
            if blocks:
                print(f"  Blocks:")
                for b in blocks:
                    print(f"    {b['color']:8s} @ zone {b['zone']} ({b['position']})")
            return

        print(f"[DigitalTwin] Unknown command: {raw}")

    def _handle_scenario(self, user_text: str):
        """Match user text against scenarios and execute tasks."""
        result = self._scenario_mgr.match(user_text)
        if not result:
            print(f"[Scenario] No matching scenario for: {user_text}")
            return

        action = result.get("action")
        reasoning = result.get("reasoning", "")
        print(f"[Scenario] {reasoning}")

        if action == "ask":
            question = result.get("question", "")
            options = result.get("options", [])
            print(f"[Scenario] Q: {question}")
            for i, opt in enumerate(options, 1):
                print(f"  {i}. {opt}")
            print("  → Enter 1 or 2:")
            return

        if action == "execute":
            tasks = result.get("tasks", [])
            for task in tasks:
                color = task["color"]
                zone = task["to_zone"]
                position = task.get("to_position", "center")
                self._block_mgr.move_block(color, zone, position)
            print(f"[Scenario] Executed {len(tasks)} task(s)")

    def _on_arrive_idle(self):
        """Default on-arrive: go to IDLE."""
        print(f"[DigitalTwin] Arrived at {self._current_goal_name} → IDLE")
        self._mode = RobotMode.IDLE

    # -------------------------------------------------------------------
    # Main tick
    # -------------------------------------------------------------------
    def _on_timeline_tick(self, event: carb.events.IEvent):
        if not self._running:
            return

        # Capture viewport for web streaming (every 10 frames)
        if not hasattr(self, "_capture_count"):
            self._capture_count = 0
        self._capture_count += 1
        if self._capture_count % 30 == 0:  # ~2fps at 60Hz tick
            self._capture_viewport()

        # Poll ROS2 OmniGraph bridge for new data
        if self._ros2_bridge:
            self._ros2_bridge.poll()

        # Always sync arm position to follow AMR
        self._sync_arm_to_amr()

        # Follow camera tracks AMR
        self._update_follow_camera()

        # Always process commands
        self._process_commands()

        # Sync vision-detected objects only after inbound docking
        if self._ros2_bridge and self._vision_spawn_enabled:
            self._sync_vision_objects()

        # Always sync robot arm joints (digital twin)
        if self._ros2_bridge:
            positions = self._ros2_bridge.get_joint_positions()
            if any(p != 0.0 for p in positions):
                self._apply_joint_positions(positions)

            # Sync gripper + attach/detach nearest object
            stroke, g_updated = self._ros2_bridge.get_gripper_stroke()
            if g_updated:
                self._sync_gripper(stroke)

        if self._mode == RobotMode.AMR_MOVING:
            self._tick_amr_moving()
        elif self._mode == RobotMode.ARM_WORKING:
            self._tick_arm_working()

    def _sync_vision_objects(self):
        """Sync detected objects from /dsr01/curobo/obstacles to Isaac Sim.

        Obstacles arrive in base_link frame (robot arm base = origin).
        We transform them to world frame using the arm's current world position.

        Instead of re-spawning every frame, we maintain persistent prims
        and just update their positions (no physics, visual only).
        """
        obstacles, updated = self._ros2_bridge.get_obstacles()
        if not updated:
            return

        if obstacles:
            # Log first sync only
            if not hasattr(self, "_vision_logged"):
                self._vision_logged = True
                print(f"[Vision] Receiving {len(obstacles)} objects, first: {obstacles[0].get('name', '?')}")

        # Get arm world transform to convert base_link → world
        if not self._manip_prim:
            return
        arm_xf = UsdGeom.Xformable(
            self._manip_prim
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        arm_pos = arm_xf.ExtractTranslation()

        color_map = {
            "red": (0.9, 0.15, 0.15),
            "green": (0.15, 0.8, 0.15),
            "yellow": (0.95, 0.85, 0.1),
        }

        # Initialize persistent vision prims dict
        if not hasattr(self, "_vision_prims"):
            self._vision_prims = {}  # name → prim_path

        # Track which prims are still active this frame
        active_names = set()

        for obs in obstacles:
            name = obs.get("name", "object")
            pos = obs.get("pos", [0, 0, 0])
            dims = obs.get("dims", [0.04, 0.04, 0.04])

            # Skip objects with no valid position
            if pos is None or all(p == 0 for p in pos):
                continue

            active_names.add(name)

            # pos is in base_link frame → convert to world
            # Apply arm's world rotation (yaw) to the local offset
            arm_rot = arm_xf.ExtractRotation()
            arm_axis = arm_rot.GetAxis()
            arm_angle = arm_rot.GetAngle()
            yaw_rad = math.radians(
                arm_angle if arm_axis[2] > 0 else -arm_angle
            )
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)
            world_x = arm_pos[0] + pos[0] * cos_y - pos[1] * sin_y
            world_y = arm_pos[1] + pos[0] * sin_y + pos[1] * cos_y
            world_z = arm_pos[2] + pos[2]

            # Determine color
            obj_color = (0.7, 0.7, 0.7)
            for color_key, rgb in color_map.items():
                if color_key in name.lower():
                    obj_color = rgb
                    break

            safe_name = name.replace(" ", "_").replace("/", "_")
            prim_path = f"/VisionObjects/{safe_name}"

            if name in self._vision_prims:
                # Update existing prim position
                prim = self._stage.GetPrimAtPath(prim_path)
                if prim:
                    prim.GetAttribute("xformOp:translate").Set(
                        Gf.Vec3d(world_x, world_y, world_z)
                    )
            else:
                # Create new visual-only prim (no physics!)
                cube = UsdGeom.Cube.Define(self._stage, prim_path)
                xf = UsdGeom.Xformable(cube.GetPrim())
                if not cube.GetPrim().GetAttribute("xformOp:translate"):
                    xf.AddTranslateOp()
                if not cube.GetPrim().GetAttribute("xformOp:scale"):
                    xf.AddScaleOp()
                cube.GetPrim().GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(world_x, world_y, world_z)
                )
                cube.GetPrim().GetAttribute("xformOp:scale").Set(
                    Gf.Vec3d(dims[0] / 2, dims[1] / 2, dims[2] / 2)
                )
                cube.GetDisplayColorAttr().Set([Gf.Vec3f(*obj_color)])
                # NO RigidBodyAPI — purely visual
                self._vision_prims[name] = prim_path

        # Remove prims for objects no longer detected
        gone = set(self._vision_prims.keys()) - active_names
        for name in gone:
            path = self._vision_prims.pop(name)
            prim = self._stage.GetPrimAtPath(path)
            if prim:
                self._stage.RemovePrim(path)

    def _tick_amr_moving(self):
        """Move AMR toward goal by interpolating position each frame."""
        goal = self._waypoints[self._current_goal_name]
        pos = self._get_amr_position()

        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.GOAL_TOLERANCE:
            # Snap to goal
            self._set_amr_position(goal)
            print(f"[DigitalTwin] AMR arrived at {self._current_goal_name}")
            if self._on_arrive_callback:
                self._on_arrive_callback()
                self._on_arrive_callback = None
        else:
            # Move toward goal
            step = min(self._amr_nav_speed * self._physics_dt, dist)
            nx, ny = dx / dist, dy / dist
            new_pos = (pos[0] + nx * step, pos[1] + ny * step, pos[2])
            self._set_amr_position(new_pos)

            # Rotate AMR to face movement direction
            yaw_deg = math.degrees(math.atan2(ny, nx))
            self._amr_root_prim.GetAttribute("xformOp:rotateXYZ").Set(
                Gf.Vec3d(0.0, 0.0, yaw_deg)
            )

    def _tick_arm_working(self):
        self._arm_work_tick_count += 1

        # Sync joints from real robot
        if self._ros2_bridge:
            positions = self._ros2_bridge.get_joint_positions()
            self._apply_joint_positions(positions)

    def _capture_viewport(self):
        """Capture Isaac Sim viewport to JPEG for web streaming."""
        try:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            capture_viewport_to_file(get_active_viewport(), "/tmp/isaacsim_frame.jpg")
        except Exception:
            pass

    def _on_stage_closing(self, event: carb.events.IEvent):
        self.stop()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    demo = CarterE0509DigitalTwin(enable_ros2=True)
    demo.start()

    while simulation_app.is_running() and demo.is_running():
        simulation_app.update()

    simulation_app.close()
