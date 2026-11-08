#pragma once

#include <deque>
#include <memory>

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

struct Ekf {
  using Ptr = std::shared_ptr<Ekf>;

  int id{0};
  double dt;
  rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
  int age{0};
  int update_num{0};
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;
  std::deque<Eigen::MatrixXd> InnoCov_list;

  explicit Ekf(double step) : dt(step) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 6);
    C.setZero(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    const double t2 = dt * dt / 2.0;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 3) = dt;
    B(4, 4) = dt;
    B(5, 5) = dt;
    C.setIdentity();
    K = C;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt.diagonal().setConstant(0.1);
    Rt.diagonal() << 0.09, 0.09, 0.09, 0.4, 0.4, 0.4;
    x.setZero(6);
  }

  void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + Qt;
  }

  void reset(const Eigen::Vector3d &z, int id_, const rclcpp::Time &stamp) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
    last_update_stamp_ = stamp;
    age = 1;
    update_num = 0;
    id = id_;
  }

  bool checkValid(const Eigen::Vector3d &z1, const Eigen::Vector3d &z2) const {
    Eigen::VectorXd z(6);
    z << z1, z2;
    const Eigen::MatrixXd gain = Sigma * C.transpose() *
      (C * Sigma * C.transpose() + Rt).inverse();
    const Eigen::VectorXd x_tmp = x + gain * (z - C * x);
    return x_tmp.tail(3).norm() <= 4.0;
  }

  void update(const Eigen::Vector3d &z1, const Eigen::Vector3d &z2,
              const rclcpp::Time &stamp) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
    last_update_stamp_ = stamp;
    ++update_num;
  }

  Eigen::Vector3d pos() const { return x.head(3); }
  Eigen::Vector3d vel() const { return x.tail(3); }
};
