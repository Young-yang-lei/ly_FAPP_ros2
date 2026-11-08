#include "mapping_manager.h"
#include <functional>
#include <cmath>
#include <iostream>
#include <cstdio>

namespace mot_mapping {

MappingRos::MappingRos(const rclcpp::NodeOptions &options)
: Node("mapping_node", options) {
  init();
}

MappingRos::~MappingRos() {std::cout << "Exit Tracker" << std::endl;}

void MappingRos::init() {
  frame_num_ = 1;
  id_ = 0;

  remaining_points_.reset(new Points);
  ikdtree_ptr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

  mp_ = std::make_unique<MapParam>();
  md_ = std::make_unique<MapData>();

  mp_->core_pts = declare_parameter<int>("dbscan_.core_pts", 4);
  mp_->tolerance = declare_parameter<double>("dbscan_.tolerance", 0.02);
  mp_->min_cluster = declare_parameter<int>("dbscan_.min_cluster", 20);
  mp_->max_cluster = declare_parameter<int>("dbscan_.max_cluster", 800);
  mp_->thresh_dist = declare_parameter<double>("detection.thresh_dist", 0.08);
  mp_->thresh_var = declare_parameter<double>("detection.thresh_var", 0.28);
  mp_->range[0] = declare_parameter<double>("lidar.range_x", 15.0);
  mp_->range[1] = declare_parameter<double>("lidar.range_y", 15.0);
  mp_->range[2] = declare_parameter<double>("lidar.range_z", 1.0);
  mp_->odom_topic = declare_parameter<std::string>("ros.odom_topic", "/odom");
  mp_->lidar_topic = declare_parameter<std::string>("ros.lidar_topic", "/livox/lidar");

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_points", 10);
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map_ros", 10);
  static_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/static_map", 10);
  edge_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/box_edge", 10);
  object_pose_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/object_pose", 10);
  states_pub_ = create_publisher<obj_state_msgs::msg::ObjectsStates>("/states", 10);

  std::cout << "INIT!" << std::endl;

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, mp_->lidar_topic, rmw_qos_profile_sensor_data));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, mp_->odom_topic, rmw_qos_profile_default));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(std::bind(&MappingRos::cloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

  ekf_predict_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MappingRos::ekfPredictCallback, this));
  map_pub_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&MappingRos::mapPubCallback, this));
}


float MappingRos::calc_dist(PointType p1, PointType p2) const {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, const std::vector<float> &box_lengths) const {
    const float x_dist = box_lengths[0];
    const float y_dist = box_lengths[1];
    const float z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

void MappingRos::pointsBodyToWorld(const PointsPtr &p_b, PointsPtr &p_w) const {
  int num = p_b->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_b->points[i].x, 
         p_b->points[i].y, 
         p_b->points[i].z;
    // p = md_->R * p + md_->T;
    if ((p - md_->T).head(2).norm() < 5.0) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

void MappingRos::pointsWorldToBody(const PointsPtr &p_w, PointsPtr &p_b, const Eigen::Vector3d &T) const {
  int num = p_w->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_w->points[i].x, 
         p_w->points[i].y, 
         p_w->points[i].z;
    p = (p - T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


void MappingRos::cloudOdomCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
                                   nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;
       
  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  // clear screen
  printf("\033[2J");
  printf("\033[1;1H");
  
  // Pointcloud Filter
  PointsPtr cloud_filtered(new Points);
  vox_.setInputCloud(latest_cloud);
  vox_.setLeafSize(0.1, 0.1, 0.1);
  vox_.filter(*cloud_filtered);

  PointsPtr cloud_world(new Points);
  PointsPtr nonfilter_pts(new Points);
  PointsPtr point_to_add(new Points);
  PointsPtr cluster_points(new Points);
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts);

  buffer_.push_back(nonfilter_pts);
  if (buffer_.size() > 1) {
    buffer_.pop_front();
  }
  for (int i = 0; i < buffer_.size(); ++i) {
    *cluster_points += *buffer_[i];
  }
  vox_.setInputCloud(cluster_points);
  vox_.setLeafSize(0.05, 0.05, 0.05);
  vox_.filter(*cluster_points);

  remaining_points_ = cluster_points;
  
  PointsPtr All_Points(new Points);

  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Filter Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  std::cout << "Input Size:" << cluster_points->size() << std::endl;
  
  sensor_msgs::msg::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox_.setInputCloud(cluster_points);
  vox_.setLeafSize(0.05, 0.05, 0.05);
  vox_.filter(*OutputPoints);

  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = "world";
  map_pub_->publish(map_ros);

  // DBSCAN Cluster
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan_.setCorePointMinPts(mp_->core_pts);
  dbscan_.setClusterTolerance(mp_->tolerance);
  dbscan_.setMinClusterSize(mp_->min_cluster);
  dbscan_.setMaxClusterSize(mp_->max_cluster);
  dbscan_.setInputCloud(cluster_points);
  dbscan_.setSearchMethod();
  dbscan_.extractNano(cluster_indices);

  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Cluster Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  if (frame_num_ < 2){
    previous_points_.push_back(cloud_world);
    frame_num_++;
    return;
  }

  int pt_size = previous_points_[0]->points.size();
  //ikd-tree
  if (ikdtree_ptr_->Root_Node == nullptr) {
    ikdtree_ptr_->Build(previous_points_[0]->points);
    previous_points_.push_back(cloud_world);
    previous_points_.pop_front();
  }
  else {
    PointVector PointNoNeedDownsample;

    for (int i = 0; i < pt_size; ++i) {
      PointType mid_point; 
      mid_point.x = floor(previous_points_[0]->points[i].x/0.1)*0.1 + 0.5 * 0.1;
      mid_point.y = floor(previous_points_[0]->points[i].y/0.1)*0.1 + 0.5 * 0.1;
      mid_point.z = floor(previous_points_[0]->points[i].z/0.1)*0.1 + 0.5 * 0.1;
      // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr_->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

      if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 || 
          fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 || 
          fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1) {
        PointNoNeedDownsample.push_back(previous_points_[0]->points[i]);
        point_to_add->points.push_back(previous_points_[0]->points[i]);
      }
    }
    // int add_point_size = ikdtree_ptr_->Add_Points(point_to_add, true);
    ikdtree_ptr_->Add_Points(PointNoNeedDownsample, false); 
    input_point_.push_back(point_to_add);
    if (input_point_.size() > 25) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point_[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr_->Delete_Points(PointDelete);
      input_point_.pop_front();
    }

    previous_points_.push_back(cloud_world);
    previous_points_.pop_front();
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "ikd-Tree Time Cost (ms)： " << compTime << std::endl;
  tic = std::chrono::high_resolution_clock::now();
  std::cout << "ikd-Tree Size: " << ikdtree_ptr_->size() << std::endl;
  // ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, ikdtree_ptr_->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr_->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;

  // sensor_msgs::msg::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloud_pub_.publish(previous_cloud);

  PointsPtr dynamic_points(new Points);

  int k = 0;
  detections_.clear();
  deleted_indices_.clear();
  for (auto& getIndices: cluster_indices) {
    PointsPtr cluster(new Points);
    PointVector PointDelete;
    double avg_dist = 0;
    Eigen::Vector3d cluster_center;
    cluster_center.setZero();
    std::vector<double> avg_buffer;
    for (auto& index : getIndices.indices) {
      cluster->points.push_back(cluster_points->points[index]);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr_->Nearest_Search(cluster_points->points[index], 3, points_near, pointSearchSqDis);
      avg_dist += sqrt(pointSearchSqDis[0]);
      avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
      cluster_center += Eigen::Vector3d(cluster_points->points[index].x, cluster_points->points[index].y, cluster_points->points[index].z);
    }

    avg_dist = avg_dist/getIndices.indices.size();
    cluster_center = cluster_center/getIndices.indices.size();
    double var_dist = 0;
    for (auto& dist : avg_buffer) {
      var_dist += (dist - avg_dist) * (dist - avg_dist) / (avg_dist * avg_dist);
    }
    var_dist = var_dist/getIndices.indices.size();
    
    bool on_track = false;
    for (int i = 0; i < trackers_.size(); ++i) {
      Eigen::Vector3d tracker_center = trackers_[i]->pos();
      double dist = sqrt((cluster_center(0) - tracker_center(0)) * (cluster_center(0) - tracker_center(0)) + 
                         (cluster_center(1) - tracker_center(1)) * (cluster_center(1) - tracker_center(1)) + 
                         (cluster_center(2) - tracker_center(2)) * (cluster_center(2) - tracker_center(2)));
      if (dist < 0.5 && trackers_[i]->age > 3 && trackers_[i]->vel().norm() > 0.2) {
        on_track = true;
        break;
      }
    }

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    pcl::PointXYZ minPt, maxPt;
	  pcl::getMinMax3D(*cluster, minPt, maxPt);

    PointType center_pt;
    center_pt.x = (maxPt.x + minPt.x)/2;
    center_pt.y = (maxPt.y + minPt.y)/2;
    center_pt.z = (maxPt.z + minPt.z)/2;

    if ((avg_dist > mp_->thresh_dist && avg_dist < 5.0 && var_dist < mp_->thresh_var) || on_track) {
      // std::cout << "on_track: " << on_track << std::endl;
      // std::cout << "var_dist: " << var_dist << std::endl;
      // std::cout << "avg_dist: " << avg_dist << std::endl;
      ObjectState state;
      state.id = k;
      state.position = cluster_center;
      state.size << maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z;
      detections_.push_back(state);
      deleted_indices_.push_back(getIndices);
      for (auto& index : getIndices.indices) {
        dynamic_points->points.push_back(cluster_points->points[index]);
      }
      // BoxPointType box;
      // box.vertex_min[0] = minPt.x-0.2; box.vertex_min[1] = minPt.y-0.2; box.vertex_min[2] = minPt.z-0.2;
      // box.vertex_max[0] = maxPt.x+0.2; box.vertex_max[1] = maxPt.y+0.2; box.vertex_max[2] = maxPt.z+0.2;
      // delete_boxes_.push_back(box);
    }
  }

  sensor_msgs::msg::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = "world";
  cloud_pub_->publish(dynamic_pts);

  // Tracking & EKF
  if(trackers_.size() == 0) {
    for(int i = 0; i < detections_.size(); ++i) {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections_[i].position, id_, this->now());
      id_++;
      trackers_.push_back(ekfPtr);
      previous_p_.push_back(detections_[i].position);
      Eigen::Vector3d v0(0,0,0);
      previous_v_.push_back(v0);
    }
    return;
  }


  obj_state_msgs::msg::ObjectsStates states;
  std::vector<std::pair<int, int>> matchedPairs;

  for (int i = 0; i < detections_.size(); ++i) {
    double min_dist = 1000000;
    int min_index = -1;
    Eigen::Vector3d det_pos = detections_[i].position;
    for (int j = 0; j < trackers_.size(); ++j) {
      trackers_[j]->age = trackers_[j]->age + 1;
      Eigen::Vector3d track_pos = trackers_[j]->pos();
      double dist = (det_pos - track_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    }

    if (min_dist < 0.8) {
      // std::cout << "id:" << trackers_[min_index]->id << std::endl;
      Eigen::Vector3d vel_detect = (detections_[i].position - previous_p_[min_index]) / 0.02;
      trackers_[min_index]->update(detections_[i].position, 0.5 * vel_detect + 0.5 * previous_v_[min_index], this->now());

      matchedPairs.push_back(std::make_pair(i, min_index));

      ObjectState state;
      state.position = trackers_[min_index]->pos();
      state.velocity = trackers_[min_index]->vel();
      obj_state_msgs::msg::State statemsg;
      statemsg.header.stamp = this->now();
      statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
      statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
      statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
      states.states.push_back(statemsg);

    } else {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections_[i].position, id_, this->now());
      id_++;
      trackers_.push_back(ekfPtr);
    }
  }
  states_pub_->publish(states);

  visualizeFunction(matchedPairs);

  for (auto it = trackers_.begin(); it != trackers_.end();) {
      // std::cout << "dt:" << (*it)->age - (*it)->update_num << std::endl;
      if ((*it)->age - (*it)->update_num > 20)
        it = trackers_.erase(it);
      else 
        it++;
  }    

  previous_p_.clear();
  previous_v_.clear();
  for (int i = 0; i < trackers_.size(); ++i ) {
    previous_p_.push_back(trackers_[i]->pos());
    previous_v_.push_back(trackers_[i]->vel());
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Tracking Time Cost (ms)： " << compTime <<std::endl;
}

void MappingRos::ekfPredictCallback() {
  if (trackers_.size() == 0)
    return;

  for (int i = 0; i < trackers_.size(); ++i) {
    trackers_[i]->predict();
  }
}

void MappingRos::mapPubCallback() {
  if (remaining_points_->points.size() == 0)
    return;
  

  // KD_TREE<PointType>::Ptr delete_tree;
  // delete_tree.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
  // delete_tree->Build(remaining_points_->points);
  // delete_tree->Delete_Point_Boxes(delete_boxes_);

  // Points static_pts;
  // delete_tree->flatten(delete_tree->Root_Node, delete_tree->PCL_Storage, NOT_RECORD);
  // static_pts.points = delete_tree->PCL_Storage;
  // static_pts.width = static_pts.points.size();
  // static_pts.height = 1;
  // static_pts.is_dense = true;


  // delete_boxes_.clear();
  // sensor_msgs::msg::PointCloud2 map_static;
  // pcl::toROSMsg(static_pts, map_static);
  // map_static.header.frame_id = "world";
  // static_map_pub_.publish(map_static);
}


double MappingRos::iou(ObjectState state1, ObjectState state2) const {
  double distance = (state1.position - state2.position).norm();
  double iou = atan(distance)*2/M_PI;
  return iou;
}

void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> &pairs) {
  visualization_msgs::msg::MarkerArray poses;
  visualization_msgs::msg::MarkerArray boxes;

  for (auto pair : pairs) {
    int detectionIndex = pair.first;
    int trackerIndex = pair.second;
    pcl::PointXYZ minPt, maxPt;
    // if (detections_[detectionIndex].position[1] - detections_[detectionIndex].size[1]/2 > 1.7 ||
    //     detections_[detectionIndex].position[1] < -2.0 ||
    //     detections_[detectionIndex].position[2] > 1.8)
    //   continue;
    minPt.x = detections_[detectionIndex].position[0] - detections_[detectionIndex].size[0]/2;
    minPt.y = detections_[detectionIndex].position[1] - detections_[detectionIndex].size[1]/2;
    minPt.z = detections_[detectionIndex].position[2] - detections_[detectionIndex].size[2]/2;
    maxPt.x = detections_[detectionIndex].position[0] + detections_[detectionIndex].size[0]/2;
    maxPt.y = detections_[detectionIndex].position[1] + detections_[detectionIndex].size[1]/2;
    maxPt.z = detections_[detectionIndex].position[2] + detections_[detectionIndex].size[2]/2;
    visualization_msgs::msg::Marker edgeMarker;
    edgeMarker.id = detectionIndex;
    edgeMarker.header.stamp = this->now();
    edgeMarker.header.frame_id = "world";
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime.sec = 0;
    edgeMarker.lifetime.nanosec = 100000000;
    edgeMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edgeMarker.action = visualization_msgs::msg::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;
    edgeMarker.scale.x = 0.1;
    geometry_msgs::msg::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker); 

    visualization_msgs::msg::Marker poseMarker;
    ObjectState state;
    state.position = trackers_[trackerIndex]->pos();
    state.velocity = trackers_[trackerIndex]->vel();
    poseMarker.id = trackerIndex+100;
    poseMarker.header.stamp = this->now();
    poseMarker.header.frame_id = "world";
    poseMarker.lifetime.sec = 0;
    poseMarker.lifetime.nanosec = 100000000;
    poseMarker.type = visualization_msgs::msg::Marker::ARROW;
    poseMarker.action = visualization_msgs::msg::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;
    poseMarker.scale.y = 0.18;
    poseMarker.scale.z = 0.30;
    poseMarker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point arrow[2];
    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];
    const double speed = state.velocity.norm();
    arrow[1] = arrow[0];
    if (speed > 0.01) {
      arrow[1].x += state.velocity[0] / speed;
      arrow[1].y += state.velocity[1] / speed;
      arrow[1].z += state.velocity[2] / speed;
    }
    // std::cout << "vx:" << state.velocity[0] << std::endl;
    // std::cout << "vy:" << state.velocity[1] << std::endl;
    // std::cout << "vz:" << state.velocity[2] << std::endl;
    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);
    if (speed > 0.01)
      poses.markers.push_back(poseMarker);
  }
  object_pose_pub_->publish(poses);
  edge_pub_->publish(boxes);

}

}
