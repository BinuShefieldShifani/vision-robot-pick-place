# System Overview & Hardware Setup

## Physical Setup

```
┌─────────────────────────────────────┐
│           Overhead View             │
│                                     │
│  [Basler Camera] (mounted overhead) │
│         │                           │
│         ▼  (field of view)          │
│  ┌──────────────────────┐           │
│  │   Conveyor / Worktable           │
│  │                                  │
│  │  [ArUco Marker]  ← placed flat   │
│  │                                  │
│  │  ○  □  △   ← objects to pick    │
│  └──────────────────────┘           │
│                                     │
│  [ABB Robot] ← picks from table,   │
│               places at wobj2       │
└─────────────────────────────────────┘
```

## Components

| Component | Details |
|---|---|
| Robot | ABB IRB series with RAPID controller |
| Gripper | Vacuum gripper (digital output: `doValve1`) |
| Camera | Basler industrial camera (GigE or USB3 Vision) |
| Marker | ArUco DICT_5X5_100, ID 0, printed at 100mm × 100mm |
| PC | Runs Python vision system, connected on 192.168.125.x subnet |

## Network Configuration

The PC and robot controller must be on the same subnet:

```
PC IP         : 192.168.125.201   (set in config.py)
Robot IP      : 192.168.125.1     (typical ABB default)
Port          : 5000
```

Configure the PC's network adapter to a static IP in the 192.168.125.x range.

## Work Objects (wobj)

The RAPID program uses two work objects:

| Variable | Purpose |
|---|---|
| `wobj1` | Pick zone — where objects are detected by camera |
| `wobj2` | Place zone — where objects are placed after picking |

These need to be re-calibrated on your physical robot using RobotStudio or the teach pendant.

## ArUco Marker Setup

1. Print the marker at **exactly 100mm × 100mm** (no scaling)
2. Place flat on the conveyor/table surface, within camera field of view
3. Ensure good lighting — avoid glare on the marker
4. The marker defines the coordinate origin (0, 0) for all detections

## Teach Pendant Operation

1. Power on robot and load `MainModule.mod`
2. Run the program — it will connect to the PC server
3. Operator is prompted: `Enter 1 for Circle, 2 for Square, 3 for Triangle`
4. Robot sends the shape name to PC, receives coordinates, executes pick & place
5. Press OK button (`diOkButton`) to confirm each cycle

## Tuning Notes

- `angle + 30` offset in triangle angle calculation is hardware-specific — re-tune for your gripper
- `angle + 55` offset for square is similarly hardware-specific
- Adjust `CONF_THRESHOLD` in `configs/config.py` if you get false positives or misses
- `BUFFER` (10px) can be increased if contour extraction is unstable
- `ARUCO_LENGTH` must match your physically printed marker size exactly
