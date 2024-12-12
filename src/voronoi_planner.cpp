#include "voronoi_planner/voronoi_planner.hpp"
#include <map_msgs/msg/detail/occupancy_grid_update__struct.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <oneapi/tbb/info.h>
#include <oneapi/tbb/task_arena.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

namespace voronoi_planner {

void VoronoiPlanner::outlineMap(unsigned char *costarr, int nx, int ny,
                                unsigned char value) {
  unsigned char *pc = costarr;
  for (int i = 0; i < nx; i++)
    *pc++ = value;
  pc = costarr + (ny - 1) * nx;

  for (int i = 0; i < nx; i++)
    *pc++ = value;
  pc = costarr;

  for (int i = 0; i < ny; i++, pc += nx)
    *pc = value;
  pc = costarr + nx - 1;

  for (int i = 0; i < ny; i++, pc += nx)
    *pc = value;
}

void VoronoiPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  if (!initialized_) {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    /*frame_id_ = frame_id;*/
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

    plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
    voronoi_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "voronoi_grid", 1);

    costmap_update_subscriber_ =
        node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "some_costmap_update_topic", 10,
            std::bind(&VoronoiPlanner::costmapUpdateCallback, this,
                      std::placeholders::_1));
    initialized_ = true;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "Voronoi Planner has already been configured. Doing nothing.");
  }
}
void VoronoiPlanner::costmapUpdateCallback(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) const {
  RCLCPP_INFO(node_->get_logger(),
              "Received occupancy grid update: x=%d, y=%d, width=%d, height=%d",
              msg->x, msg->y, msg->width, msg->height);
}

void VoronoiPlanner::clearRobotCell(const geometry_msgs::msg::Pose &goal_pose,
                                    unsigned int mx, unsigned int my) {
  if (!initialized_) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "The planner has not been initialized yet, but it is being used.");
    return;
  }
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

// TODO: Make Plan Service here
// VoronoiPlanner::makePlanService

void VoronoiPlanner::mapToWorld(double mx, double my, double &wx, double &wy) {
  float convert_offset_ = 0;
  wx = costmap_->getOriginX() +
       (mx + convert_offset_) * costmap_->getResolution();
  wy = costmap_->getOriginY() +
       (my + convert_offset_) * costmap_->getResolution();
}

bool VoronoiPlanner::worldToMap(double wx, double wy, double &mx, double &my) {
  double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();

  if (wx < origin_x || wy < origin_x) {
    return false;
  }

  float convert_offset_ = 0;
  mx = (wx - origin_x) / resolution - convert_offset_;
  my = (wy - origin_y) / resolution - convert_offset_;

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  // NOTE: if there is an error, the values of mx, and my are not chnaged. Could
  // cause bugs?
  return false;
}

nav_msgs::msg::Path
VoronoiPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                           const geometry_msgs::msg::PoseStamped &goal) {
  return createPlan(start, goal, default_tolerance_);
}

nav_msgs::msg::Path
VoronoiPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                           const geometry_msgs::msg::PoseStamped &goal,
                           double tolerance) {
  boost::mutex::scoped_lock lock(mutex_);

  nav_msgs::msg::Path empty_path;
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Planner has not been initialised but it is being called");
    empty_path.poses.clear();
    return empty_path;
  }

  nav_msgs::msg::Path path;
  path.poses.clear();
  std::string global_frame = frame_id_;
  // ... inside your function that checks poses
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer->lookupTransform(
        global_frame, goal.header.frame_id, node_->get_clock()->now());
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Could not transform %s to %s: %s",
                 goal.header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return empty_path;
  }

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
  double start_x, start_y, goal_x, goal_y;

  if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
    RCLCPP_WARN(node_->get_logger(),
                "The robots position is off the global costmap. Planning will "
                "fail. Make sure it is localized properly");
    return empty_path;
  }
  worldToMap(wx, wy, start_x, start_y);

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;
  if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1.0,
                         "The goal send is off the global costmap. Planning to "
                         "this goal is not possible");
    return empty_path;
  }
  worldToMap(wx, wy, goal_x, goal_y);

  // clear the starting cell in the costmap because it wont be a obstacle
  tf2::Stamped<geometry_msgs::msg::Pose> start_pose;
  tf2::fromMsg(start, start_pose);
  clearRobotCell(start_pose, start_x_i, start_y_i);

  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  outlineMap(costmap_->getCharMap(), nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);

  bool **map = NULL;

  int sizeX = costmap_->getSizeInCellsY(), sizeY = costmap_->getSizeInCellsY();

  map = new bool *[sizeX];
  RCLCPP_INFO(node_->get_logger(), "Map Size is %dx%d", sizeX, sizeY);

  auto t = node_->get_clock()->now();
  auto t_b = node_->get_clock()->now();

  for (int x = 0; x < sizeX; x++) {
    (map)[x] = new bool[sizeY];
  }

  for (int y = sizeY - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX; x++) {
      unsigned char c = costmap_->getCost(x, y);

      if (c == nav2_costmap_2d::FREE_SPACE ||
          c == nav2_costmap_2d::NO_INFORMATION)
        (map)[x][y] = false; // cell is free
      else
        (map)[x][y] = true; // cell is occupied
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Time to convert map: %f sec",
              (node_->get_clock()->now() - t).seconds());

  bool doPrune = true;

  // initialize voronoi object with the map
  t = node_->get_clock()->now();
  RCLCPP_INFO(node_->get_logger(), "voronoi.initializeMap");
  voronoi_.initializeMap(sizeX, sizeY, map);
  RCLCPP_INFO(node_->get_logger(), "Time for initialization of map: %f sec",
              (node_->get_clock()->now() - t).seconds());

  t = node_->get_clock()->now();
  RCLCPP_INFO(node_->get_logger(), "voronoi.update");
  voronoi_.update(); // update distance map and Voronoi Diagram
  RCLCPP_INFO(node_->get_logger(), "Time for map update: %f sec",
              (node_->get_clock()->now() - t).seconds());

  t = node_->get_clock()->now();
  RCLCPP_INFO(node_->get_logger(), "voronoi.prune");
  if (doPrune)
    voronoi_.prune();
  RCLCPP_INFO(node_->get_logger(), "Time  for map prune: %f sec",
              (node_->get_clock()->now() - t).seconds());

  t = node_->get_clock()->now();
  voronoi_.visualize("~/map.ppm");
  RCLCPP_INFO(node_->get_logger(), "visualize done");

  std::cerr << "Generated Initial frame. \n";

  std::vector<std::pair<float, float>> path1;
  std::vector<std::pair<float, float>> path2;
  std::vector<std::pair<float, float>> path3;

  //    start_x = 10;
  //    start_y = 100;
  //    goal_x = 300;
  //    goal_y = 330;

  std::cout << "start_x,start_y " << start_x << " " << start_y << std::endl;
  std::cout << "goal_x,goal_y " << goal_x << " " << goal_y << std::endl;

  bool res1 = false, res2 = false, res3 = false;

  if (!voronoi_.isVoronoi(goal_x, goal_y)) {
    //        path3 = findPath( goal, init, A, 0, 1 );
    res3 = findPath(&path3, goal_x, goal_y, start_x, start_y, &voronoi_, 0, 1);
    std::cout << "findPath 3 res " << res3 << std::endl;
    //        goal = path3(end,:);
    goal_x = std::get<0>(path3[path3.size() - 1]);
    goal_y = std::get<1>(path3[path3.size() - 1]);

    std::cout << "voronoi.isVoronoi(goal_x,goal_y) "
              << voronoi_.isVoronoi(goal_x, goal_y) << std::endl;

    //        path3 = flipud(path3);
    std::reverse(path3.begin(), path3.end());
  }

  if (!voronoi_.isVoronoi(start_x, start_y)) {
    res1 = findPath(&path1, start_x, start_y, goal_x, goal_y, &voronoi_, 0, 1);
    std::cout << "findPath 1 res " << res1 << std::endl;
    start_x = std::get<0>(path1[path1.size() - 1]);
    start_y = std::get<1>(path1[path1.size() - 1]);

    std::cout << "voronoi.isVoronoi(start_x,start_y) "
              << voronoi_.isVoronoi(start_x, start_y) << std::endl;
  }

  res2 = findPath(&path2, start_x, start_y, goal_x, goal_y, &voronoi_, 1, 0);
  std::cout << "findPath 2 res " << res2 << std::endl;

  if (!(res1 && res2 && res3)) {
    RCLCPP_INFO(node_->get_logger(), "Failed to find full path");
  }
  //    else
  //    {

  //    path = [path;path2;path3];
  path1.insert(path1.end(), path2.begin(), path2.end());
  path1.insert(path1.end(), path3.begin(), path3.end());

  for (int i = 0; i < path1.size(); i++) {
    int x = std::get<0>(path1[i]);
    int y = std::get<1>(path1[i]);

    //        std::cout << "[" << x << "; " <<
    //                     y << "]" << std::endl;

    if (x > 0 && y > 0)
      map[x][y] = 1;
  }

  //    if(smooth_path_){
  //        smoothPath(&path1);
  //    }

  //    visualize("/tmp/plan.ppm", &voronoi_, map, &path1);

  for (int i = 0; i < path1.size(); i++) {

    geometry_msgs::msg::PoseStamped new_goal = goal;
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, 1.54);

    new_goal.pose.position.x = std::get<0>(path1[i]);
    new_goal.pose.position.y = std::get<1>(path1[i]);

    mapToWorld(new_goal.pose.position.x, new_goal.pose.position.y,
               new_goal.pose.position.x, new_goal.pose.position.y);

    //        std::cout << "[" << new_goal.pose.position.x << "; " <<
    //                     new_goal.pose.position.y << "]" << std::endl;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();
    path.poses.push_back(new_goal);
  }

  // add orientations if needed
  //    orientation_filter_->processPath(start, plan);

  //    }

  RCLCPP_ERROR(node_->get_logger(), "\nTime to get plan: %f sec\n",
               (node_->get_clock()->now() - t_b).seconds());

  // publish the plan for visualization purposes
  publishPlan(path);

  if (publish_voronoi_grid_) {
    publishVoronoiGrid(&voronoi_);
  }

  //    delete potential_array_;

  for (int x = 0; x < sizeX; x++) {
    delete[] map[x];
  }
  delete[] map;

  return path;
}
} // namespace voronoi_planner
