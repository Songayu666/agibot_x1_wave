#include "control_module/utilities.h"

// 节流器
bool Throttler(const time_point<high_resolution_clock> now, time_point<high_resolution_clock> &last, const milliseconds interval) {
  auto elapsed = now - last;

  if (elapsed >= interval) {
    last = now;
    return true;
  }
  return false;
}

// 数字低通滤波器
template <typename T>
digital_lp_filter<T>::digital_lp_filter(T w_c, T t_s) {
  Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
  Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
  Lpf_in1 = 0, Lpf_in2 = 0, Lpf_in3 = 0, Lpf_out1 = 0, Lpf_out2 = 0;
  wc = w_c;
  ts = t_s;
  update();
}

template <typename T>
void digital_lp_filter<T>::update(void) {
  double den = 2500 * ts * ts * wc * wc + 7071 * ts * wc + 10000;

  Lpf_in1 = 2500 * ts * ts * wc * wc / den;
  Lpf_in2 = 5000 * ts * ts * wc * wc / den;
  Lpf_in3 = 2500 * ts * ts * wc * wc / den;
  Lpf_out1 = -(5000 * ts * ts * wc * wc - 20000) / den;
  Lpf_out2 = -(2500 * ts * ts * wc * wc - 7071 * ts * wc + 10000) / den;
}

template <typename T>
digital_lp_filter<T>::~digital_lp_filter(void) {}

template <typename T>
void digital_lp_filter<T>::input(T lpf_in) {
  lpf_out = Lpf_in1 * lpf_in + Lpf_in2 * Lpf_in_prev[0] + Lpf_in3 * Lpf_in_prev[1] +  // input component
            Lpf_out1 * Lpf_out_prev[0] + Lpf_out2 * Lpf_out_prev[1];                  // output component
  Lpf_in_prev[1] = Lpf_in_prev[0];
  Lpf_in_prev[0] = lpf_in;
  Lpf_out_prev[1] = Lpf_out_prev[0];
  Lpf_out_prev[0] = lpf_out;
}

template <typename T>
T digital_lp_filter<T>::output(void) {
  return lpf_out;
}

template <typename T>
void digital_lp_filter<T>::set_ts(T t_s) {
  ts = t_s;
  update();
}

template <typename T>
void digital_lp_filter<T>::set_wc(T w_c) {
  wc = w_c;
  update();
}

template <typename T>
void digital_lp_filter<T>::clear(void) {
  Lpf_in_prev[1] = 0;
  Lpf_in_prev[0] = 0;
  Lpf_out_prev[1] = 0;
  Lpf_out_prev[0] = 0;
}

template <typename T>
void digital_lp_filter<T>::init(T init_data) {
  Lpf_in_prev[1] = init_data;
  Lpf_in_prev[0] = init_data;
  Lpf_out_prev[1] = init_data;
  Lpf_out_prev[0] = init_data;
}

template class digital_lp_filter<double>;
template class digital_lp_filter<float>;

// 插值器
void Interpolator::Init(const std::vector<std::vector<double>> &to_interpolate_data,
                        double max_velocity, double max_acceleration,
                        double max_jerk) {
  construct_flag_ = false;
  data_.clear();
  if (to_interpolate_data.size() < 2 || to_interpolate_data[0].size() < 2) {
    throw std::runtime_error("A trajectory needs at least two non-empty waypoints.");
  }
  const size_t row_size = to_interpolate_data.front().size();
  for (const auto& row : to_interpolate_data) {
    if (row.size() != row_size) {
      throw std::runtime_error("All trajectory rows must have the same number of joints.");
    }
    if (row.front() <= 0.0) {
      throw std::runtime_error("Trajectory segment duration must be positive.");
    }
  }
  if (max_velocity <= 0.0 || max_acceleration <= 0.0 || max_jerk <= 0.0) {
    throw std::runtime_error("Trajectory motion limits must be positive.");
  }
  max_velocity_ = max_velocity;
  max_acceleration_ = max_acceleration;
  max_jerk_ = max_jerk;
  for (size_t ii = 0; ii < to_interpolate_data.size() - 1; ++ii) {
    std::vector<double> start(to_interpolate_data[ii].begin() + 1, to_interpolate_data[ii].begin() + to_interpolate_data[ii].size());
    std::vector<double> end(to_interpolate_data[ii + 1].begin() + 1, to_interpolate_data[ii + 1].begin() + to_interpolate_data[ii + 1].size());
    // auto interpolated_points = linearInterpolate(start, end, to_interpolate_data[ii][0]);
    auto interpolated_points = RuckigInterpolate(start, end, to_interpolate_data[ii][0]);
    data_.insert(data_.end(), interpolated_points.begin(), interpolated_points.end());
  }
  current_row_ = 0;
  construct_flag_ = true;
}

bool Interpolator::GetNextPoint(std::vector<double> &next_points) {
  if (current_row_ >= data_.size() || !construct_flag_) {
    return false;
  }
  next_points = data_[current_row_];
  ++current_row_;
  return true;
}

std::vector<std::vector<double>> Interpolator::linearInterpolate(std::vector<double> start, std::vector<double> end, double duration) {
  std::vector<std::vector<double>> interpolated;
  array_t start_array = Eigen::Map<array_t>(start.data(), start.size());
  array_t end_array = Eigen::Map<array_t>(end.data(), end.size());
  array_t step = (end_array - start_array) / (duration * 1000 - 1);
  for (int i = 0; i < duration * 1000; ++i) {
    array_t interpolated_point = start_array + i * step;
    std::vector<double> interpolated_point_vec(interpolated_point.data(), interpolated_point.data() + interpolated_point.size());
    interpolated.push_back(interpolated_point_vec);
  }
  return interpolated;
}

std::vector<std::vector<double>> Interpolator::RuckigInterpolate(
    const std::vector<double> &start, const std::vector<double> &end,
    double duration) {
  std::vector<std::vector<double>> interpolated;
  if (start.size() != end.size() || start.empty()) {
    throw std::runtime_error("Trajectory waypoint dimensions do not match.");
  }
  ruckig::Ruckig<ruckig::DynamicDOFs> otg(start.size(), 1.0 / 1000.0);
  ruckig::InputParameter<ruckig::DynamicDOFs> input(start.size());
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(start.size());

  input.current_position = start;
  input.current_velocity.assign(start.size(), 0.0);
  input.current_acceleration.assign(start.size(), 0.0);
  input.target_position = end;
  input.target_velocity.assign(start.size(), 0.0);
  input.target_acceleration.assign(start.size(), 0.0);
  input.max_velocity.assign(start.size(), max_velocity_);
  input.max_acceleration.assign(start.size(), max_acceleration_);
  input.max_jerk.assign(start.size(), max_jerk_);
  input.minimum_duration = duration;

  while (true) {
    const auto result = otg.update(input, output);
    if (result < 0) {
      throw std::runtime_error("Ruckig failed to generate a trajectory segment.");
    }
    interpolated.push_back(output.new_position);
    if (result == ruckig::Result::Finished) {
      break;
    }
    output.pass_to_input(input);
  }
  return interpolated;
}
