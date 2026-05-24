# madodom_ws

MAD-ICP-inspired 3D LiDAR odometry for ROS2 Humble.
Two packages: `madodom_core` (Eigen-only C++ library) and `madodom_ros` (ROS2 wrapper + offline runner).

---

## Build

```bash
cd ~/madodom_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Offline Evaluation (no RViz needed)

Reads a bag directly, runs odometry at full CPU speed, writes a TUM trajectory file.

### Step 1 — Generate estimate

```bash
source /opt/ros/humble/setup.bash && source ~/madodom_ws/install/setup.bash

# Dataset 1 (still sensor)
ros2 run madodom_ros madodom_offline ~/dataset1/bag/ ~/dataset1/madodom_estimate.txt 1100

# Dataset 2 (wheeled robot)
ros2 run madodom_ros madodom_offline ~/dataset2/bag/ ~/dataset2/madodom_estimate.txt 1100

# Dataset 3 (robot dog on stairs)
ros2 run madodom_ros madodom_offline ~/dataset3/bag/ ~/dataset3/madodom_estimate.txt 1100
```

Arguments: `<bag_dir> <output_tum> [intensity_min]`
- `intensity_min` default is 1100 — drop points below this intensity value.

### Step 2 — Plot with evo

```bash
source ~/evo_env/bin/activate

# Fix ground truth whitespace (one-time, skip if already done)
cd ~/dataset2 && awk '{$1=$1; print}' optimized_traj.txt > tmp.txt && mv tmp.txt optimized_traj.txt
cd ~/dataset3 && awk '{$1=$1; print}' optimized_traj.txt > tmp.txt && mv tmp.txt optimized_traj.txt

# Dataset 1 — xy view (pose is static, not very interesting)
cd ~/dataset1
evo_traj tum madodom_estimate.txt --ref optimized_traj.txt -p --plot_mode xy

# Dataset 2 — xy view (wheeled robot loop)
cd ~/dataset2
evo_traj tum madodom_estimate.txt --ref optimized_traj.txt -p --plot_mode xy

# Dataset 3 — xyz view (shows stair climb in Z)
cd ~/dataset3
evo_traj tum madodom_estimate.txt --ref optimized_traj.txt -p --plot_mode xyz
```

TUM format: `timestamp tx ty tz qx qy qz qw`

---

## Live Visualization (RViz)

Three terminals required.

**Terminal 1 — node + RViz:**
```bash
cd ~/madodom_ws && source install/setup.bash
ros2 launch madodom_ros madodom.launch.py rviz:=true
```

**Terminal 2 — ground truth publisher:**
```bash
source /opt/ros/humble/setup.bash && source ~/madodom_ws/install/setup.bash

# Pick the dataset you are testing:
python3 ~/madodom_ws/src/madodom_ros/scripts/publish_gt.py ~/dataset2/optimized_traj.txt
# python3 ~/madodom_ws/src/madodom_ros/scripts/publish_gt.py ~/dataset3/optimized_traj.txt
```

**Terminal 3 — bag playback:**
```bash
# Dataset 2:
ros2 bag play ~/dataset2/bag/ --clock \
  --qos-profile-overrides-path ~/madodom_ws/src/madodom_ros/config/qos_override.yaml

# Dataset 3:
ros2 bag play ~/dataset3/bag/ --clock \
  --qos-profile-overrides-path ~/madodom_ws/src/madodom_ros/config/qos_override.yaml
```

RViz displays:
| Colour | Topic | Meaning |
|--------|-------|---------|
| Green line | `/path` | madodom estimate |
| Red line | `/gt_path` | ground truth |
| Points | `/ouster/points` | live LiDAR scan |

To speed up playback add `-r 2.0` (2× speed) to the bag play command.

---

## Datasets

| Dataset | Description | Scans | Duration |
|---------|-------------|-------|----------|
| `~/dataset1/bag/` | Still sensor | 365 | ~36 s |
| `~/dataset2/bag/` | Wheeled robot, ~30×36 m loop | 2840 | ~284 s |
| `~/dataset3/bag/` | Robot dog on stairs, 7 m climb | 3441 | ~344 s |

Ground truth: `optimized_traj.txt` in each dataset folder.
Reference MAD-ICP estimate: `estimate.txt` in each dataset folder.
