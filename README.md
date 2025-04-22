# Voronoi Planner for Nav2

The Voronoi Planner is a global planner plugin for the Nav2 planner server. Its primary feature is the ability to generate global plans that ensure the robot navigates in the middle of a corridor, even while taking turns. This is achieved using Voronoi diagrams.

## Features

- Generates paths that maximize clearance from obstacles.
- Ensures smooth navigation through narrow corridors.
- Integrates seamlessly with the Nav2 framework.

## Future Goals

- [ ] Complete rewrite of dynamicVoronoi.
    - Modern CPP implementation
    - Better data structure implementation
- [ ] Make GVD generation multi-threaded on large maps.
- [ ] OpenMP integration maybe for parallelism?

## Installation

1. Clone the repository into your ROS2 workspace:

   ```bash
   mkdir -p ros2_ws/src
   cd ros2_ws/
   git clone https://github.com/DhruvPotdar/voronoi_planner_nav2 src/voronoi_planner_nav2
   ```

2. Build the workspace:

   ```bash
   colcon build
   ```

3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Usage with Nav2

To use the Voronoi Planner as the global planner in Nav2, update your `nav2_params.yaml` file to include the following configuration:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: [ "VoronoiPlanner"]
    VoronoiPlanner:
      plugin: "voronoi_planner/VoronoiPlanner"
```

Ensure that the `plugin` field for `VoronoiPlanner` matches the class name defined in the `voronoi_planner.xml` file.


## Code Overview

### Core Components

1. **`VoronoiPlanner`**:

   - Implements the `nav2_core::GlobalPlanner` interface.
   - Provides methods for configuration, activation, deactivation, and cleanup.
   - Computes paths using Voronoi diagrams.

2. **`DynamicVoronoi`**:

   - Computes and updates a distance map and Voronoi diagram.
   - Provides methods to initialize maps, add/remove obstacles, and update the Voronoi diagram.

3. **`BucketPrioQueue`**:
   - A priority queue optimized for integer coordinates and squared distances.
   - Used internally by the Voronoi algorithm for efficient processing.



## License

This project is licensed under the [MIT License](LICENSE).

## Credits
- [Dynamic Voronoi](https://github.com/frontw/dynamicvoronoi.git)
- [Voronoi Planner For ROS1](https://github.com/frontw/voronoi_planner.git)
