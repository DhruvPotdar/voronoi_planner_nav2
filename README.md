# Nav2 Voronoi Planner

The VoronoiPlanner is a global planner for the Nav2 planner server. It uses the (<https://github.com/frontw/dynamicvoronoi.git)(https://github.com/frontw/dynamicvoronoi.git>) to calculate a Generalised Voronoi Diagram from the map. It is mostly equivalent to its counterpart in ROS1.

Its main feature is the ability to give global plans that ensure that the robot navigates in the middle of a corridor even while taking turns.
