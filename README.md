# Voronoi Planner for Nav2

The Voronoi Planner is a global planner plugin for the Nav2 planner server. Its primary feature is the ability to generate global plans that ensure the robot navigates in the middle of a corridor, even while taking turns. This is achieved using Voronoi diagrams.

## Features

- Generates paths that maximize clearance from obstacles.
- Ensures smooth navigation through narrow corridors.
- Integrates seamlessly with the Nav2 framework.

## Future Goals

- [ ] Complete rewrite of dynamicVoronoi.
- [ ] Make GVD generation multi-threaded on large maps.
- [ ] OpenMP integration maybe for parallelism?

## Installation

1. Clone the repository into your ROS2 workspace:

   ```bash
   git clone <repository_url> src/voronoi_planner_nav2
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
    planner_plugins: ["GridBased", "VoronoiPlanner"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
    VoronoiPlanner:
      plugin: "voronoi_planner/VoronoiPlanner"
```

Ensure that the `plugin` field for `VoronoiPlanner` matches the class name defined in the `voronoi_planner.xml` file.

## Parameters

The Voronoi Planner supports the following parameters:

- **`expected_planner_frequency`**: The frequency at which the planner is expected to run.
- **`planner_plugins`**: A list of planner plugins to load. Ensure `VoronoiPlanner` is included here.
- **`plugin`**: The plugin class name for the Voronoi Planner (`voronoi_planner/VoronoiPlanner`).

Additional parameters can be added to customize the behavior of the planner. Refer to the source code for more details.

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

### Key Methods

- **`createPlan`**: Generates a path given a start and goal pose.
- **`findPath`**: Computes a path using the Voronoi diagram.
- **`initializeMap`**: Initializes the map with obstacles.
- **`update`**: Updates the Voronoi diagram to reflect changes in the map.

## Visualization

The Voronoi Planner includes a method to visualize the Voronoi diagram and distance map. Use the `visualize` method in `DynamicVoronoi` to output the diagram as a `.ppm` file.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or support, please contact the maintainer at [dhruvpotdar29@gmail.com].
