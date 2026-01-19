# Biped Robot - Project Memory

## Current Status (2026-01-19)

**✅ Robot is UPRIGHT and all parts visible!**

12 body parts as colored cubes in standing humanoid pose:
- Torso (RED, center) - upright stance
- Head (YELLOW) - above torso
- L-Shoulder, L-Elbow, L-Hip, L-Knee, L-Ankle (GREEN) - left side standing
- R-Shoulder, R-Elbow, R-Hip, R-Knee, R-Ankle (BLUE) - right side standing

All parts are siblings under /Robot with absolute world positions - robot is standing upright!

---

## Robot File

```
/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda
```

**Test with GUI:**
```bash
cd /home/kenpeter/work/biped_robot
./run_isaac.sh test_humanoid_visible.py
```

**Regenerate USD:**
```bash
./run_isaac.sh setup_isaac_sim_robot.py
```

---

## Fix History

### 2026-01-19: Changed to CUBES in FLAT sibling structure

**Problem:** Robot used spheres instead of cubes, and nested hierarchy caused parts to snap together.

**Solution:** Complete rewrite of setup_isaac_sim_robot.py:
- Changed from UsdGeom.Sphere to UsdGeom.Cube (robot.png shows cubes!)
- All 12 body parts created as SIBLINGS under /Robot (not nested)
- Each part has absolute world position coordinates
- Prevents PhysX from snapping parts together at single location
- Joints connect between siblings using body0/body1 relationships

**USD Structure (FLAT):**
```
/Robot/
├── torso (RED, z=0, size=0.24) [ArticulationRoot]
├── head (YELLOW, z=0.3, size=0.16)
├── l_shoulder (GREEN, x=-0.3, z=0.1, size=0.12)
├── l_elbow (GREEN, x=-0.6, z=0.1, size=0.10)
├── r_shoulder (BLUE, x=0.3, z=0.1, size=0.12)
├── r_elbow (BLUE, x=0.6, z=0.1, size=0.10)
├── l_hip (GREEN, x=-0.15, z=-0.3, size=0.12)
├── l_knee (GREEN, x=-0.15, z=-0.6, size=0.10)
├── l_ankle (GREEN, x=-0.15, z=-0.9, size=0.10)
├── r_hip (BLUE, x=0.15, z=-0.3, size=0.12)
├── r_knee (BLUE, x=0.15, z=-0.6, size=0.10)
└── r_ankle (BLUE, x=0.15, z=-0.9, size=0.10)
```

**Key Technical Insight:**
- NESTED hierarchy (parent→child links) causes PhysX to snap parts together due to transform accumulation
- FLAT sibling structure with absolute positions keeps parts spread out correctly
- Joints reference sibling paths via body0/body1 relationships

### 2026-01-19 (Earlier): Attempted nested hierarchy

**Problem:** Nested parent-child link structure caused "disjointed body transforms" and parts snapped to single location.

**Learning:** PhysX articulation works better with flat sibling structure for USD-based robots.

---

## Quick Commands

```bash
# Regenerate robot from robot.png structure
./run_isaac.sh setup_isaac_sim_robot.py

# Test robot with GUI
./run_isaac.sh test_humanoid_visible.py

# Check structure (should show CUBES)
grep -E "(def Xform|def Cube|xformOp:translate)" models/humanoid_articulated.usda

# Count cubes (should be 12)
grep -c "def Cube" models/humanoid_articulated.usda
```

---

## File Structure

```
biped_robot/
├── models/
│   ├── humanoid_articulated.usda   # Robot USD with 12 cubes
│   └── robot.png                    # Reference image for structure
│
├── setup_isaac_sim_robot.py        # Generates USD from robot.png
├── test_humanoid_visible.py        # Test script with GUI
├── run_isaac.sh                     # Isaac Sim launcher
│
├── README.md                        # User documentation
├── MEMORY.md                        # This file - development notes
└── CLAUDE.md                        # Claude AI instructions
```

---

## Known Issues

**Resolved:**
- ✅ Parts snapping together → use flat sibling structure (2026-01-19)
- ✅ Using spheres instead of cubes → changed to UsdGeom.Cube (2026-01-19)
- ✅ Nested hierarchy causing transform issues → flat structure (2026-01-19)
- ✅ xformOpOrder missing (2026-01-19 earlier)

**None currently!**

---

*Last updated: 2026-01-19*
