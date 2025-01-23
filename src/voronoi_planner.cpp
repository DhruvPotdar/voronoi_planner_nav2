#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "voronoi_planner/voronoi_planner.hpp"
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <memory>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    RCLCPP_INFO(logger_, "====================================================="
                         "=================================================");
    planner_ = std::make_unique<voronoi_planner::VoronoiPlanner>();
    parent_node_ = parent;
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    auto logger_ = node_->get_logger();
    clock_ = node_->get_clock();
    planner_->costmap_ = costmap_ros->getCostmap();
    frame_id_ = costmap_ros->getGlobalFrameID();

    plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
    voronoi_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "voronoi_grid", 1);

    costmap_update_subscriber_ =
        node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "/global_costmap/costmap_updates", 10,
            std::bind(&VoronoiPlanner::costmapUpdateCallback, this,
                      std::placeholders::_1));
    initialized_ = true;
  } else {
    RCLCPP_WARN(logger_,
                "Voronoi Planner has already been configured. Doing nothing.");
  }

  double origin_x = planner_->costmap_->getOriginX();
  double origin_y = planner_->costmap_->getOriginY();
  double size_x = planner_->costmap_->getSizeInMetersX();
  double size_y = planner_->costmap_->getSizeInMetersY();

  RCLCPP_INFO(
      logger_,
      "Costmap bounds: origin_x: %f, origin_y: %f, size_x: %f, size_y: %f",
      origin_x, origin_y, size_x, size_y);
}
void VoronoiPlanner::costmapUpdateCallback(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) const {
  RCLCPP_INFO(logger_, "++++++++++++++++++++++++++++++++++++++++++++++++");
  RCLCPP_INFO(logger_,
              "Received occupancy grid update: x=%d, y=%d, width=%d, height=%d",
              msg->x, msg->y, msg->width, msg->height);
}

void VoronoiPlanner::clearRobotCell(unsigned int mx, unsigned int my) {
  if (!initialized_) {
    RCLCPP_ERROR(
        logger_,
        "The planner has not been initialized yet, but it is being used.");
    return;
  }
  planner_->costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

void VoronoiPlanner::mapToWorld(double mx, double my, double &wx, double &wy) {
  float convert_offset_ = 0;
  double resolution = planner_->costmap_->getResolution();
  wx = planner_->costmap_->getOriginX() + ((mx + convert_offset_) * resolution);
  wy = planner_->costmap_->getOriginY() + ((my + convert_offset_) * resolution);
}

void VoronoiPlanner::cleanup() {
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type voronoi_planner",
              name_.c_str());
  planner_.reset();
}

void VoronoiPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s of type voronoi_planner",
              name_.c_str());
  // Add callback for dynamic parameters
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &VoronoiPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void VoronoiPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type voronoi_planner",
              name_.c_str());
  auto node = parent_node_.lock();
  if (node && dyn_params_handler_) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

bool VoronoiPlanner::worldToMap(double wx, double wy, double &mx, double &my) {
  double origin_x = planner_->costmap_->getOriginX(),
         origin_y = planner_->costmap_->getOriginY();
  double resolution = planner_->costmap_->getResolution();

  if (wx < origin_x || wy < origin_x) {
    RCLCPP_DEBUG(logger_,
                 "World coordinates [%.2f, %.2f] are outside map bounds", wx,
                 wy);
    return false;
  }

  float convert_offset_ = 0;
  mx = (wx - origin_x) / resolution - convert_offset_;
  my = (wy - origin_y) / resolution - convert_offset_;

  if (mx < 0 || my < 0 || mx >= planner_->costmap_->getSizeInCellsX() ||
      my >= planner_->costmap_->getSizeInCellsY()) {
    // RCLCPP_DEBUG(logger_,
    //              "Computed map coordinates [%.2f, %.2f] are outside map
    //              bounds", mx, my);
    return false;
  }

  return true;
}

nav_msgs::msg::Path
VoronoiPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                           const geometry_msgs::msg::PoseStamped &goal) {
  boost::mutex::scoped_lock lock(mutex_);

  nav_msgs::msg::Path empty_path;
  if (!initialized_) {
    RCLCPP_ERROR(logger_,
                 "Planner has not been initialised but it is being called");
    empty_path.poses.clear();
    return empty_path;
  }

  nav_msgs::msg::Path path;
  path.poses.clear();
  std::string global_frame = frame_id_;

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped =
        tf_->lookupTransform(global_frame, goal.header.frame_id, clock_->now());
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "Could not transform %s to %s: %s",
                 goal.header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return empty_path;
  }

  // Check if start is very close to goal
  double distance = std::hypot(start.pose.position.x - goal.pose.position.x,
                               start.pose.position.y - goal.pose.position.y);

  // Define a small threshold (e.g., 0.1 meters)
  if (distance < 0.1) {
    // Create a plan with just the current position
    nav_msgs::msg::Path current_pos_plan;
    current_pos_plan.header.frame_id = frame_id_;
    current_pos_plan.header.stamp = node_->now();

    // Add the current position as the only pose in the plan
    geometry_msgs::msg::PoseStamped current_pose = start;
    current_pos_plan.poses.push_back(current_pose);

    return current_pos_plan;
  }

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
  double start_x, start_y, goal_x, goal_y;

  // auto origin_x = planner_->costmap_->getOriginX();
  // auto origin_y = planner_->costmap_->getOriginY();
  // auto map_height = planner_->costmap_->getResolution();
  if (!planner_->costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
    RCLCPP_WARN(logger_,
                "The robot's position [%.2f, %.2f] is off the global costmap. "
                "Cost map bounds: [%.2f, %.2f] to [%.2f, %.2f]",
                wx, wy, planner_->costmap_->getOriginX(),
                planner_->costmap_->getOriginY(),
                planner_->costmap_->getOriginX() +
                    planner_->costmap_->getSizeInMetersX(),
                planner_->costmap_->getOriginY() +
                    planner_->costmap_->getSizeInMetersY());
    return empty_path;
  }
  worldToMap(wx, wy, start_x, start_y);

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;
  if (!planner_->costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1.0,
                         "The goal send is off the global costmap. Planning to "
                         "this goal is not possible");
    return empty_path;
  }
  worldToMap(wx, wy, goal_x, goal_y);
  // clear the starting cell in the costmap because it wont be a obstacle
  clearRobotCell(start_x_i, start_y_i);

  RCLCPP_DEBUG(logger_, "Planning from [%d, %d] to [%d, %d]", start_x_i,
               start_y_i, goal_x_i, goal_y_i);

  int nx = planner_->costmap_->getSizeInCellsX(),
      ny = planner_->costmap_->getSizeInCellsY();
  outlineMap(planner_->costmap_->getCharMap(), nx, ny,
             nav2_costmap_2d::LETHAL_OBSTACLE);

  bool **map = NULL;

  int sizeX = planner_->costmap_->getSizeInCellsY(),
      sizeY = planner_->costmap_->getSizeInCellsY();

  map = new bool *[sizeX];
  RCLCPP_INFO(logger_, "Map Size is %dx%d", sizeX, sizeY);

  auto t = clock_->now();
  auto t_b = clock_->now();

  for (int x = 0; x < sizeX; x++) {
    (map)[x] = new bool[sizeY];
  }

  for (int y = sizeY - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX; x++) {
      unsigned char c = planner_->costmap_->getCost(x, y);

      if (c == nav2_costmap_2d::FREE_SPACE ||
          c == nav2_costmap_2d::NO_INFORMATION)
        (map)[x][y] = false; // cell is free
      else
        (map)[x][y] = true; // cell is occupied
    }
  }
  // RCLCPP_INFO(logger_, "Current time %ld", clock_->now().nanoseconds());
  // RCLCPP_INFO(logger_, "Current time %ld", t.nanoseconds());
  // RCLCPP_INFO(logger_, "Time to convert map: %f sec",
  //             (clock_->now() - t).seconds());

  bool doPrune = true;

  // initialize voronoi object with the map
  t = clock_->now();
  // RCLCPP_INFO(logger_, "voronoi.initializeMap");
  voronoi_.initializeMap(sizeX, sizeY, map);
  // RCLCPP_INFO(logger_, "Time for initialization of map: %f sec",
  // (clock_->now() - t).seconds());

  t = clock_->now();
  // RCLCPP_INFO(logger_, "voronoi.update");
  voronoi_.update(); // update distance map and Voronoi Diagram
  // RCLCPP_INFO(logger_, "Time for map update: %f sec",
  // (clock_->now() - t).seconds());

  t = clock_->now();
  // RCLCPP_INFO(logger_, "voronoi.prune");
  if (doPrune)
    voronoi_.prune();
  // RCLCPP_INFO(logger_, "Time  for map prune: %f sec",
  // (clock_->now() - t).seconds());

  std::cerr << "Generated Initial frame. \n";

  std::vector<std::pair<float, float>> path1;
  std::vector<std::pair<float, float>> path2;
  std::vector<std::pair<float, float>> path3;

  //    start_x = 10;
  //    start_y = 100;
  //    goal_x = 300;
  //    goal_y = 330;

  // std::cout << "start_x,start_y " << start_x << " " << start_y << std::endl;
  // std::cout << "goal_x,goal_y " << goal_x << " " << goal_y << std::endl;

  bool res1 = false, res2 = false, res3 = false;

  if (!voronoi_.isVoronoi(goal_x, goal_y)) {
    //        path3 = findPath( goal, init, A, 0, 1 );
    res3 = findPath(&path3, goal_x, goal_y, start_x, start_y, &voronoi_, 0, 1);
    // std::cout << "findPath 3 res " << res3 << std::endl;
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
    // std::cout << "findPath 1 res " << res1 << std::endl;
    start_x = std::get<0>(path1[path1.size() - 1]);
    start_y = std::get<1>(path1[path1.size() - 1]);

    std::cout << "voronoi.isVoronoi(start_x,start_y) "
              << voronoi_.isVoronoi(start_x, start_y) << std::endl;
  }

  res2 = findPath(&path2, start_x, start_y, goal_x, goal_y, &voronoi_, 1, 0);
  std::cout << "findPath 2 res " << res2 << std::endl;

  if (!(res1 && res2 && res3)) {
    RCLCPP_INFO(logger_, "Failed to find full path");
  }
  //    else
  //    {

  //    path = [path;path2;path3];
  path1.insert(path1.end(), path2.begin(), path2.end());
  path1.insert(path1.end(), path3.begin(), path3.end());

  for (unsigned long i = 0; i < path1.size(); i++) {
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

  for (unsigned long i = 0; i < path1.size(); i++) {

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

  RCLCPP_ERROR(logger_, "Time to get plan: %f sec",
               (clock_->now() - t_b).seconds());

  // publish the plan for visualization purposes
  path.header.stamp = node_->now();
  path.header.frame_id = frame_id_;

  if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "The plan is empty.");
  }

  auto last = path.poses.back();

  RCLCPP_INFO(logger_, "Publishing plan");
  plan_pub_->publish(path);

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

bool VoronoiPlanner::findPath(std::vector<std::pair<float, float>> *path,
                              int init_x, int init_y, int goal_x, int goal_y,
                              DynamicVoronoi *voronoi, bool check_is_voronoi,
                              bool stop_at_voronoi) {
  // Plan from any path in the grid to a voronoi cell, so that the rest of the
  // path can be found

  // RCLCPP_INFO(logger_,
  //             "init_x %d, init_y %d, goal_x %d, goal_y %d, "
  //             "check_is_voronoi_cell % d, stop_at_voronoi % d ",
  //             init_x, init_y, goal_x, goal_y, check_is_voronoi,
  //             stop_at_voronoi);
  RCLCPP_INFO(logger_, "isVoronoi(init) %d; isVoronoi(goal) %d",
              voronoi->isVoronoi(init_x, init_y),
              voronoi->isVoronoi(goal_x, goal_y));
  // available movements (actions) of the robot on the grid
  std::vector<std::pair<int, int>> delta;
  delta.push_back({-1, 0});  // go up
  delta.push_back({0, -1});  // go left
  delta.push_back({1, 0});   // go down
  delta.push_back({0, 1});   // go right
  delta.push_back({-1, -1}); // up and left
  delta.push_back({-1, 1});  // up and right
  delta.push_back({1, -1});  // down and left
  delta.push_back({1, 1});   // down and right

  // cost of movement
  float cost = 1;

  // grid size
  unsigned int sizeX = voronoi->getSizeX();
  unsigned int sizeY = voronoi->getSizeY();

  // closed cells grid (same size as map grid)
  bool **closed = NULL;
  closed = new bool *[sizeX];
  for (unsigned int x = 0; x < sizeX; x++) {
    (closed)[x] = new bool[sizeY];
  }

  for (auto y = static_cast<int>(sizeY) - 1; y >= 0; y--) {
    for (unsigned int x = 0; x < sizeX; x++) {
      (closed)[x][y] = false;
    }
  }

  // heuristic = zeros(szA(1), szA(2));
  // for(i=1:szA(1))
  //     for(j=1:szA(2))
  //         heuristic(i,j) = norm( [i - goal(1); j - goal(2)] );
  //     end
  // end

  // actions (number of delta's row) cells grid (same size as map grid)
  int **action = NULL;
  action = new int *[sizeX];
  for (unsigned int x = 0; x < sizeX; x++) {
    (action)[x] = new int[sizeY];
  }
  for (auto y = static_cast<int>(sizeY) - 1; y >= 0; y--) {
    for (unsigned int x = 0; x < sizeX; x++) {
      (action)[x][y] = -1;
    }
  }

  // set current cell
  int x = init_x;
  int y = init_y;

  // set cost
  float g = 0;

  // f = heuristic() + g;

  // vector of open (for possible expansion) nodes
  std::vector<std::tuple<float, int, int>> open;
  open.push_back(std::make_tuple(g, x, y));

  // path found flag
  bool found = false;
  // no solution could be found flag
  bool resign = false;

  while (!found && !resign) {
    if (open.size() == 0) {
      resign = true;
      // ! TODO: Check this is what is needed to be done or something else
      std::cout << "Path not found??" << std::endl;
      return false;
    } else {
      // sort open by cost
      sort(open.begin(), open.end());
      reverse(open.begin(), open.end());
      // get node with lowest cost
      std::tuple<float, int, int> next = open[open.size() - 1];
      open.pop_back();
      g = std::get<0>(next);
      x = std::get<1>(next);
      y = std::get<2>(next);

      // check, whether the solution is found (we are at the goal)
      if (stop_at_voronoi) {
        // if stop_at_voronoi is set, we stop, when get path to any voronoi cell
        if (voronoi->isVoronoi(x, y)) {
          found = true;
          goal_x = x;
          goal_y = y;
          continue;
        }
      } else {
        if (x == goal_x && y == goal_y) {
          found = true;
          continue;
        }
      }
      for (unsigned int i = 0; i < delta.size(); i++) {
        // expansion
        unsigned int x2 = x + std::get<0>(delta[i]);
        unsigned int y2 = y + std::get<1>(delta[i]);

        // check new node to be in grid bounds
        if (x2 < sizeX && y2 < sizeY) {
          // check new node not to be in obstacle
          if (voronoi->isOccupied(x2, y2)) {
            continue;
          }
          // check new node was not early visited
          if (closed[x2][y2]) {
            continue;
          }

          // check new node is on Voronoi diagram
          if (!voronoi->isVoronoi(x2, y2) && check_is_voronoi) {
            continue;
          }

          float g2 = g + cost;
          //                        f2 = heuristic(x2,y2) + g2;
          open.push_back(std::make_tuple(g2, x2, y2));
          closed[x2][y2] = true;
          action[x2][y2] = i;
        }
      }
    }
  }

  // Make reverse steps from goal to init to write path
  x = goal_x;
  y = goal_y;

  int i = 0;
  path->clear();

  while (x != init_x || y != init_y) {
    path->push_back({x, y});
    i++;

    int x2 = x - std::get<0>(delta[action[x][y]]);
    int y2 = y - std::get<1>(delta[action[x][y]]);

    x = x2;
    y = y2;
  }

  reverse(path->begin(), path->end());

  for (unsigned int x = 0; x < sizeX; x++) {
    delete[] closed[x];
  }
  delete[] closed;

  for (unsigned int x = 0; x < sizeX; x++) {
    delete[] action[x];
  }
  delete[] action;

  return true;
}

void VoronoiPlanner::smoothPath(std::vector<std::pair<float, float>> *path) {
  // Make a deep copy of path into new path
  std::vector<std::pair<float, float>> newpath = *path;

  float tolerance = 0.00001;
  float change = tolerance;

  if (path->size() < 2)
    return;

  while (change >= tolerance) {

    change = 0.0;
    for (unsigned long i = 1; i < path->size() - 1; i++) {
      float aux_x = newpath[i].first;
      float aux_y = newpath[i].second;

      float newpath_x = newpath[i].first +
                        weight_data_ * (path->at(i).first - newpath[i].first);
      float newpath_y = newpath[i].second +
                        weight_data_ * (path->at(i).second - newpath[i].second);

      newpath_x = newpath_x +
                  weight_smooth_ * (newpath[i - 1].first +
                                    newpath[i + 1].first - (2.0 * newpath_x));
      newpath_y = newpath_y +
                  weight_smooth_ * (newpath[i - 1].second +
                                    newpath[i + 1].second - (2.0 * newpath_y));

      change = change + fabs(aux_x - newpath_x);
      change = change + fabs(aux_y - newpath_y);

      newpath[i] = std::make_pair(newpath_x, newpath_y);
    }
  }
  *path = newpath;
}

/*void VoronoiPlanner::publishPlan(const nav_msgs::msg::Path &path) {*/
/**/
/*  if (!initialized_) {*/
/*    RCLCPP_ERROR(*/
/*        logger_,*/
/*        "The planner has not been initialized yet, but it is being used.");*/
/*    return;*/
/*  }*/
/**/
/*  if (!path.poses.empty()) {*/
/*    plan_pub_->publish(path);*/
/*  } else {*/
/*    RCLCPP_WARN(logger_, "The plan is empty");*/
/*  }*/
/*}*/
rcl_interfaces::msg::SetParametersResult
VoronoiPlanner::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    /*const auto &type = parameter.get_type();*/
    /*const auto &name = parameter.get_name();*/
    /*if (type == ParameterType::PARAMETER_INTEGER) {*/
    /*  if (name == name_ + ".how_many_corners") {*/
    /*    planner_->how_many_corners_ = parameter.as_int();*/
    /*  }*/
    /*  if (name == name_ + ".terminal_checking_interval") {*/
    /*    planner_->terminal_checking_interval_ = parameter.as_int();*/
    /*  }*/
    /*} else if (type == ParameterType::PARAMETER_DOUBLE) {*/
    /*  if (name == name_ + ".w_euc_cost") {*/
    /*    planner_->w_euc_cost_ = parameter.as_double();*/
    /*  } else if (name == name_ + ".w_traversal_cost") {*/
    /*    planner_->w_traversal_cost_ = parameter.as_double();*/
    /*  }*/
    /*} else if (type == ParameterType::PARAMETER_BOOL) {*/
    /*  if (name == name_ + ".use_final_approach_orientation") {*/
    /*    use_final_approach_orientation_ = parameter.as_bool();*/
    /*  } else if (name == name_ + ".allow_unknown") {*/
    /*    planner_->allow_unknown_ = parameter.as_bool();*/
    /*  }*/
    /*}*/
  }

  result.successful = true;
  return result;
}
void VoronoiPlanner::publishVoronoiGrid(DynamicVoronoi *voronoi) {
  int nx = planner_->costmap_->getSizeInCellsX(),
      ny = planner_->costmap_->getSizeInCellsY();

  RCLCPP_WARN(logger_, "costmap sx = %d,sy = %d, voronoi sx = %d, sy = %d", nx,
              ny, voronoi->getSizeX(), voronoi->getSizeY());

  double resolution = planner_->costmap_->getResolution();
  nav_msgs::msg::OccupancyGrid grid;
  // Publish Whole Grid
  grid.header.frame_id = frame_id_;
  grid.header.stamp = node_->now();
  grid.info.resolution = resolution;

  grid.info.width = nx;
  grid.info.height = ny;

  double wx, wy;
  planner_->costmap_->mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution / 2;
  grid.info.origin.position.y = wy - resolution / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(nx * ny);

  for (int x = 0; x < nx; x++) {
    for (int y = 0; y < ny; y++) {
      if (voronoi->isVoronoi(x, y))
        grid.data[x + y * nx] = static_cast<signed char>(128);
      else
        grid.data[x + y * nx] = 0;
    }
  }
  RCLCPP_INFO(logger_, "Publishing voronoi grid as a map topic");
  voronoi_grid_pub_->publish(grid);
}
} // namespace voronoi_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlanner,
                       nav2_core::GlobalPlanner)
