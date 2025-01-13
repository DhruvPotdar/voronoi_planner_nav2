#ifndef VORONOI_PLANNER__PLANNER_CORE_HPP_
#define VORONOI_PLANNER__PLANNER_CORE_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/thread/pthread/mutex.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/detail/occupancy_grid_update__struct.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/path.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/transform_datatypes.h>

#include <memory>
#include <string>

#include "dynamic_voronoi.hpp"

namespace voronoi_planner {

class VoronoiPlanner : public nav2_core::GlobalPlanner {
public:
  /**
   * @brief Default constructor for the VoronoiPlanner object
   */
  VoronoiPlanner() = default;

  /**
   * @brief  Constructor for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  frame_id Frame of the costmap
   */
  VoronoiPlanner(std::string name, nav2_costmap_2d::Costmap2D *costmap,
                 std::string frame_id);

  /**
   * @brief Destructor for the VoronoiPlanner object
   */
  ~VoronoiPlanner() override = default;

  /**
   * @brief Initialization function for the VoronoiPlanner
   * @param name The name of this planner
   * @param tf A shared pointer to the tf2 buffer
   * @param costmap_ros A shared pointer to the costmap
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Loading unloading and cleanup functions for the plugin
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Given a start and goal pose, return a plan
   * @param start The start pose
   * @param goal The goal pose
   * @return A vector of poses representing the plan
   */
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

  /**
   * @brief Compute path using Voronoi diagram
   * @param path Output path
   * @param init_x Starting x coordinate
   * @param init_y Starting y coordinate
   * @param goal_x Goal x coordinate
   * @param goal_y Goal y coordinate
   * @param voronoi Voronoi diagram
   * @param check_is_voronoi_cell Whether to check Voronoi cell
   * @param stop_at_voronoi Whether to stop at Voronoi cell
   * @return Whether path finding was successful
   */
  bool findPath(std::vector<std::pair<float, float>> *path, int init_x,
                int init_y, int goal_x, int goal_y, DynamicVoronoi *voronoi,
                bool check_is_voronoi_cell, bool stop_at_voronoi);

  /**
   * @brief Smooth the computed path
   * @param path Path to be smoothed
   */
  void smoothPath(std::vector<std::pair<float, float>> *path);

  /*bool makePlanService(nav_msgs::GetPlan::Request& req,
   * nav_msgs::GetPlan::Response& resp);*/

  void publishPlan(const nav_msgs::msg::Path &path);

  void publishVoronoiGrid(DynamicVoronoi *voronoi);

protected:
  // ROS2 specific members
  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voronoi_grid_pub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      costmap_update_subscriber_;
  // Store a copy of the current costmap. Called by createPlan
  nav2_costmap_2d::Costmap2D *costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<voronoi_planner::VoronoiPlanner> planner_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("VoronoiPlanner")};
  bool initialized_ = false;
  std::string frame_id_ = "map";
  std::string name_;
  // Existing configuration parameters
  bool publish_voronoi_grid_ = false;
  bool smooth_path_ = true;
  float weight_data_ = 0.5;
  float weight_smooth_ = 0.1;

private:
  // Existing coordinate transformation methods
  void mapToWorld(double mx, double my, double &wx, double &wy);
  bool worldToMap(double wx, double wy, double &mx, double &my);
  void clearRobotCell(unsigned int mx, unsigned int my);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  double planner_window_x_, planner_window_y_, default_tolerance_;
  boost::mutex mutex_;
  /*rclcpp::Service<???> make_plan_srv_;*/

  // Voronoi diagram
  DynamicVoronoi voronoi_;

  // Map processing
  void outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value);
  unsigned char *cost_array_;
  unsigned int start_x_, start_y_, end_x_, end_y_;
  void costmapUpdateCallback(
      const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) const;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      dyn_params_handler_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

} // namespace voronoi_planner

#endif // VORONOI_PLANNER__PLANNER_CORE_HPP_
