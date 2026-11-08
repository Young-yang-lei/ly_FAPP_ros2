#ifndef MOT_MAPPING_MAPPING_MANAGER_HPP_
#define MOT_MAPPING_MAPPING_MANAGER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <obj_state_msgs/msg/objects_states.hpp>
#include <obj_state_msgs/msg/state.hpp>

#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "DBSCAN_kdtree.h"
#include "target_ekf.hpp"
#include "ikd_Tree.h"
#include "ikd_Tree_impl.h"

using PointType = pcl::PointXYZ;
using Points = pcl::PointCloud<PointType>;
using PointsPtr = Points::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct MapParam {
  int core_pts{4};
  double tolerance{0.02};
  int min_cluster{20};
  int max_cluster{800};
  double thresh_dist{0.08};
  double thresh_var{0.28};
  Eigen::Vector3d range{15.0, 15.0, 1.0};
  std::string odom_topic{"/odom"};
  std::string lidar_topic{"/livox/lidar"};
};

struct MapData {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d T{Eigen::Vector3d::Zero()};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObjectState {
  int id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
};

namespace mot_mapping {

class MappingRos final : public rclcpp::Node {
public:
  explicit MappingRos(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MappingRos() override;
  void init();

private:
  using SyncPolicyCloudOdom = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

  void cloudOdomCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void ekfPredictCallback();
  void mapPubCallback();
  void visualizeFunction(const std::vector<std::pair<int, int>> &pairs);
  double iou(ObjectState state1, ObjectState state2) const;

  float calc_dist(PointType p1, PointType p2) const;
  void generate_box(BoxPointType &boxpoint, const PointType &center_pt,
                    const std::vector<float> &box_lengths) const;
  void pointsBodyToWorld(const PointsPtr &p_b, PointsPtr &p_w) const;
  void pointsWorldToBody(const PointsPtr &p_w, PointsPtr &p_b,
                         const Eigen::Vector3d &translation) const;

  pcl::VoxelGrid<PointType> vox_;
  DBSCANKdtreeCluster<PointType> dbscan_;
  KD_TREE<PointType>::Ptr ikdtree_ptr_;
  PointsPtr remaining_points_;
  int frame_num_{1};
  std::deque<PointsPtr> previous_points_;
  std::deque<PointsPtr> buffer_;
  std::deque<PointsPtr> input_point_;
  std::vector<Eigen::Vector3d> previous_p_;
  std::vector<Eigen::Vector3d> previous_v_;
  std::vector<pcl::PointIndices> deleted_indices_;
  std::vector<BoxPointType> delete_boxes_;
  std::vector<std::shared_ptr<Ekf>> trackers_;
  std::vector<ObjectState> detections_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_pose_pub_;
  rclcpp::Publisher<obj_state_msgs::msg::ObjectsStates>::SharedPtr states_pub_;
  rclcpp::TimerBase::SharedPtr ekf_predict_timer_;
  rclcpp::TimerBase::SharedPtr map_pub_timer_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> sync_cloud_odom_;

  std::unique_ptr<MapData> md_;
  std::unique_ptr<MapParam> mp_;
  int id_{0};
};

}  // namespace mot_mapping

#endif  // MOT_MAPPING_MAPPING_MANAGER_HPP_
