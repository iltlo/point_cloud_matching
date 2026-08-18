# point_cloud_matching

A **ROS 2** point cloud matching package that performs coarse-to-fine registration using [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) (coarse) and [small_gicp](https://github.com/koide3/small_gicp) (fine), with Open3D-based preprocessing.

## Features

- **Coarse registration** via KISS-Matcher (voxelized FPFH + RANSAC)
- **Fine registration** via small_gicp (VGICP / GICP)
- **Preprocessing pipeline**: range filtering + voxel downsampling (Open3D)
- **Service-based API** — call a ROS 2 service with two PCD file paths, get back a 6-DoF transform
- **Visualization node** that publishes matched clouds as `PointCloud2` topics for RViz
- **Configurable** via YAML parameter file

## Requirements

| Dependency | Purpose |
|---|---|
| [ROS 2 Humble](https://docs.ros.org/en/humble/) (or newer) | Framework |
| Python 3.8+ | Runtime |
| [Open3D](https://www.open3d.org/) | Point cloud I/O & preprocessing |
| [small_gicp](https://github.com/koide3/small_gicp) | Fine registration |
| [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) | Coarse registration (installed via pip) |
| NumPy, SciPy | Math utilities |

## Quick Start

### 1. Set Up a Virtual Environment

This package requires **NumPy < 2** for compatibility. Create a venv inside your workspace:

```bash
cd ~/pointcloud_ws
python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install "numpy<2" scipy open3d small_gicp kiss-matcher
```

> **Why `--system-site-packages`?** This lets the venv inherit ROS 2's system packages (e.g. `rclpy`) while isolating NumPy and the matching libraries. You should `source venv/bin/activate` in every terminal before running this package.

### 2. Clone & Build

```bash
cd ~/pointcloud_ws/src
git clone <this-repo-url> point_cloud_matching
cd ~/pointcloud_ws
colcon build --packages-select point_cloud_matching --symlink-install
source install/setup.bash
```

> ROS 2 needs to be sourced before building. Make sure you've run `source /opt/ros/humble/setup.bash` (or your distribution's equivalent).

## Usage

### Launch the Matching Service

```bash
ros2 launch point_cloud_matching point_cloud_matching.launch.py
```

This starts the `matching_node` with parameters from `config/params.yaml`.

### Call the Service

```bash
ros2 service call /match_clouds point_cloud_matching/srv/MatchClouds \
  "{reference_path: '/path/to/reference.pcd', target_path: '/path/to/target.pcd'}"
```

The service returns:

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | Whether matching succeeded |
| `message` | `string` | Status or error info |
| `fitness_score` | `float64` | Overlap quality (higher = better) |
| `rmse` | `float64` | Root-mean-square error (meters) |
| `elapsed_ms` | `float64` | Time taken |
| `result_transform` | `TransformStamped` | 6-DoF transform (target → reference) |

#### Optional Parameters

```
voxel_resolution  # Override downsampling resolution (default: from params.yaml)
skip_coarse       # Skip coarse matching, go straight to fine (default: false)
initial_guess     # Provide an initial transform guess (TransformStamped)
```

### Visualization

```bash
ros2 launch point_cloud_matching visualize.launch.py
```

This opens RViz and publishes matched clouds to `/viz/reference_cloud` and `/viz/source_cloud`. Call the visualizer's service:

```bash
ros2 service call /visualize_match point_cloud_matching/srv/MatchClouds \
  "{reference_path: '/path/to/reference.pcd', target_path: '/path/to/target.pcd'}"
```

## Configuration

Edit `config/params.yaml`:

```yaml
matching_node:
  ros__parameters:
    default_voxel_resolution: 0.05   # Downsampling voxel size (m)
    max_range: 80.0                  # Max point distance from centroid
    min_range: 0.5                   # Min point distance from centroid

    kiss_matcher:
      voxel_size: 0.3                # Coarse voxel size

    fine_matching:
      method: "VGICP"                # VGICP or GICP
      max_iterations: 100
      max_correspondence_distance: 1.0
      voxel_resolution: 1.0
      num_threads: 4
      num_neighbors: 20

    quality:
      min_fitness_score: 0.3         # Min acceptable overlap
      max_rmse: 0.5                  # Max acceptable RMSE (m)
```

## Package Structure

```
point_cloud_matching/
├── config/
│   ├── params.yaml                  # Node parameters
│   └── matching_viz.rviz            # RViz config for visualization
├── launch/
│   ├── point_cloud_matching.launch.py
│   └── visualize.launch.py
├── point_cloud_matching/
│   ├── matching_engine.py           # coarse + fine matching logic
│   ├── matching_node.py             # ROS 2 service server node
│   ├── match_visualizer_node.py     # RViz visualization node
│   └── preprocessing.py             # Open3D filters & downsampling
├── srv/
│   └── MatchClouds.srv              # Service interface definition
├── CMakeLists.txt
└── package.xml
```

## License

MIT