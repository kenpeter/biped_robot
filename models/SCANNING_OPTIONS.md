# 3D Scanning Options for Robot Hardware

## Option 1: Smartphone Photogrammetry (FREE)

**Apps:**
- **Polycam** (iOS/Android) - Free tier available
- **Scaniverse** (iOS) - Free
- **RealityScan** (iOS/Android) - Free

**Process:**
1. Take 50-100 photos of robot from all angles
2. App processes into 3D mesh
3. Export as OBJ/GLB
4. Import to Blender → clean up → export USD

**Pros:** Free, decent quality, works with your phone
**Cons:** 20-30 min per scan, needs good lighting

---

## Option 2: Intel RealSense (You might already have this!)

**Hardware:** Intel RealSense D435/D455 (~$200-400)
**Software:**
```bash
# Install RealSense SDK
sudo apt install librealsense2-utils

# Scan with Open3D
pip install open3d pyrealsense2
```

**Process:**
```python
import pyrealsense2 as rs
import open3d as o3d

# Capture point cloud
# Reconstruct mesh
# Export to USD
```

**Pros:** High quality, programmable, useful for robot vision too
**Cons:** Need to buy hardware (but useful for SLAM/vision later)

---

## Option 3: Depth Camera on Jetson

**If you have a depth camera connected to Jetson:**
- Intel RealSense
- Orbbec Astra
- Kinect

You can scan the robot with itself!

---

## Option 4: Manufacturer CAD Files (EASIEST!)

**Check if Hiwonder provides CAD files:**
```
# Common robot kit formats:
- STL files (for 3D printing)
- STEP/IGES (CAD format)
- URDF (ROS robot description)
```

**Where to look:**
- Hiwonder website downloads
- Robot kit CD/USB drive
- GitHub repositories
- Email support: support@hiwonder.com

**If they have URDF → Convert to USD:**
```bash
# Many ROS → USD converters exist
# Or manually extract mesh files
```

---

## Option 5: LiDAR Scanner (EXPENSIVE)

**Hardware:** iPhone/iPad Pro with LiDAR (2020+)
**Apps:** 3D Scanner App, Polycam

**Pros:** Very fast, accurate
**Cons:** Need iPhone Pro ($1000+)

---

## Recommended Approach for Your Robot

### Quick Method (Today):
1. **Contact Hiwonder** - Ask for CAD/STL/URDF files for your 15 DOF robot
2. If they provide files → convert to USD
3. Run `measure_robot.py` to get joint limits
4. Update USD with correct joint ranges

### Better Method (Next week):
1. Buy Intel RealSense D435i (~$200)
   - Use for scanning robot
   - Later use for vision/SLAM on robot
2. Scan robot parts individually
3. Assemble in Blender with correct joints
4. Export to USD

### Free Method (This week):
1. Use **Polycam app** on your phone
2. Scan robot (takes ~30 min)
3. Import OBJ to Blender
4. Clean up mesh, add joints
5. Export to USD

---

## Example: Using Polycam

1. **Download Polycam** (iOS/Android)
2. **Scan mode:** Photo mode (not LiDAR)
3. **Take photos:**
   - Walk around robot in circle
   - 50-100 photos from different heights
   - Overlap each photo by 50%
4. **Process:** App creates 3D model
5. **Export:** GLB or OBJ format
6. **Import to Blender:**
   ```bash
   blender --python import_scan.py
   ```

---

## Blender Cleanup Script

After scanning, you need to:
1. Remove background/floor
2. Separate into parts (head, arms, legs, etc.)
3. Add joint axes
4. Set physics properties
5. Export to USD

I can create a script to help with this!

---

## My Recommendation

**For a 15 DOF humanoid kit from Hiwonder:**

1. **First**, email Hiwonder support asking for CAD files
   - Many kits include STL/URDF
   - This is by far the fastest method

2. **If no CAD files**, use Polycam on your phone
   - Free, works well for small robots
   - 30-minute investment

3. **Future investment:** Intel RealSense D435i
   - Useful for robot too (obstacle detection, SLAM)
   - Better scans than phone

Want me to:
1. Create a Polycam import script?
2. Check Hiwonder website for CAD files?
3. Create a RealSense scanning script?
