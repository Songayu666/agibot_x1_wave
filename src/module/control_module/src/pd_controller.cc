#include "control_module/pd_controller.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

namespace xyber_x1_infer::rl_control_module {

PDController::PDController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles) {}

void PDController::Init(const YAML::Node &cfg_node) {
  // 初始化 joint_names_
  joint_names_.clear();
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  // printf("joint_names_: ");
  // for (const auto &name : joint_names_) {
  //   printf("%s ", name.c_str());
  // }
  // printf("\n");
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);
  start_joint_angles_.resize(joint_names_.size(), 0.0);

  // 初始化 joint_conf_
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());
  // std::cout << "init_state: " << joint_conf_.init_state.transpose() << std::endl;
  // std::cout << "stiffness: " << joint_conf_.stiffness.transpose() << std::endl;
  // std::cout << "damping: " << joint_conf_.damping.transpose() << std::endl;

  // 其他 PD 参数
  if (cfg_node["is_keep_controller"]) {
    is_keep_controller_ = cfg_node["is_keep_controller"].as<bool>();
  }

  if (cfg_node["plan_conf"]) {
    is_plan_controller_ = true;
    to_interpolate_data_ = cfg_node["plan_conf"]["trajectory_interpolator"].as<std::vector<std::vector<double>>>();
    if (cfg_node["plan_conf"]["max_velocity"]) {
      plan_max_velocity_ = cfg_node["plan_conf"]["max_velocity"].as<double>();
    }
    if (cfg_node["plan_conf"]["max_acceleration"]) {
      plan_max_acceleration_ = cfg_node["plan_conf"]["max_acceleration"].as<double>();
    }
    if (cfg_node["plan_conf"]["max_jerk"]) {
      plan_max_jerk_ = cfg_node["plan_conf"]["max_jerk"].as<double>();
    }
    if (cfg_node["plan_conf"]["lower_limits"] &&
        cfg_node["plan_conf"]["upper_limits"]) {
      plan_lower_limits_ = cfg_node["plan_conf"]["lower_limits"].as<std::vector<double>>();
      plan_upper_limits_ = cfg_node["plan_conf"]["upper_limits"].as<std::vector<double>>();
      if (plan_lower_limits_.size() != joint_names_.size() ||
          plan_upper_limits_.size() != joint_names_.size()) {
        throw std::runtime_error("plan joint limit width must match joint_list");
      }
      for (size_t joint = 0; joint < joint_names_.size(); ++joint) {
        if (plan_lower_limits_[joint] > plan_upper_limits_[joint]) {
          throw std::runtime_error("plan lower limit exceeds upper limit");
        }
      }
    }
    for (const auto& row : to_interpolate_data_) {
      if (row.size() != joint_names_.size() + 1) {
        throw std::runtime_error("trajectory_interpolator width must match joint_list");
      }
      for (size_t joint = 0; joint < joint_names_.size() &&
                             !plan_lower_limits_.empty(); ++joint) {
        if (row[joint + 1] < plan_lower_limits_[joint] ||
            row[joint + 1] > plan_upper_limits_[joint]) {
          throw std::runtime_error("trajectory waypoint exceeds configured joint limits");
        }
      }
    }
    to_interpolate_data_.insert(to_interpolate_data_.begin(), std::vector<double>{0.0}); // 在开头和结尾插入两个空的数组
    to_interpolate_data_.push_back(std::vector<double>{0.0});
  }
}

void PDController::RestartController() {
  {
    std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
    start_joint_angles_ = joint_state_data_.position;
  }
  trans_mode_percent_ = 0.0;

  if (is_plan_controller_) {
    // 在开头和结尾插入当前时刻的关节角度
    std::vector<double> temp_data = start_joint_angles_;
    temp_data.insert(temp_data.begin(), 1.0); // 1.0 表示插值时间
    to_interpolate_data_[0] = temp_data;
    to_interpolate_data_.back() = temp_data;
    trajectory_generator_.Init(to_interpolate_data_, plan_max_velocity_,
                               plan_max_acceleration_, plan_max_jerk_);
  }
}

void PDController::Update() {
  if (is_keep_controller_) {
    return;
  } else if (is_plan_controller_) {
    // trajectory_generator_.GetNextPoint(generated_joint_angles_);
    trajectory_generator_.GetNextPoint(start_joint_angles_);
    return;
  }

  trans_mode_percent_ += 1.0 / trans_mode_duration_s_ / 1000;
  trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
}

my_ros2_proto::msg::JointCommand PDController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());

  for (int ii = 0; ii < joint_names_.size(); ii++) {
    double pos_des;
    if (is_keep_controller_) {
      pos_des = start_joint_angles_[ii];
    } else if (is_plan_controller_) {
      // pos_des = generated_joint_angles_[ii];
      pos_des = start_joint_angles_[ii];
    } else {
      pos_des = start_joint_angles_[ii] * (1 - trans_mode_percent_) + joint_conf_.init_state(ii) * trans_mode_percent_;
    }

    joint_cmd.position[ii] = pos_des;
    joint_cmd.velocity[ii] = 0.0;
    joint_cmd.effort[ii] = 0.0;
    joint_cmd.stiffness[ii] = joint_conf_.stiffness(ii);
    joint_cmd.damping[ii] = joint_conf_.damping(ii);
  }
  return joint_cmd;
}

}  // namespace xyber_x1_infer::rl_control_module
