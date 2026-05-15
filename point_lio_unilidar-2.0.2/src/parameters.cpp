#include "parameters.h"

bool is_first_frame = true;

double lidar_end_time = 0.0;
double first_lidar_time = 0.0;
double time_con = 0.0;

double last_timestamp_lidar = -1.0;
double last_timestamp_imu = -1.0;

int pcd_index = 0;

std::string lid_topic;
std::string imu_topic;

bool prop_at_freq_of_imu;
bool check_satu;
bool con_frame;
bool cut_frame;

bool use_imu_as_input;
bool space_down_sample;
bool publish_odometry_without_downsample;

int init_map_size;
int con_frame_num;

double match_s;
double satu_acc;
double satu_gyro;
double cut_frame_time_interval;

float plane_thr;

double filter_size_surf_min;
double filter_size_map_min;
double fov_deg;

double cube_len;
float DET_RANGE;

bool imu_en;
bool gravity_align;
bool non_station_start;

double imu_time_inte;

double laser_point_cov;
double acc_norm;

double vel_cov;
double acc_cov_input;
double gyr_cov_input;

double gyr_cov_output;
double acc_cov_output;
double b_gyr_cov;
double b_acc_cov;

double imu_meas_acc_cov;
double imu_meas_omg_cov;

int lidar_type;
int pcd_save_interval;

std::vector<double> gravity_init;
std::vector<double> gravity;

std::vector<double> extrinT;
std::vector<double> extrinR;

bool runtime_pos_log;
bool pcd_save_en;
bool path_en;
bool extrinsic_est_en = true;

bool scan_pub_en;
bool scan_body_pub_en;

shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

template <typename T>
void getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, T &value, const T &default_value)
{
  node->declare_parameter<T>(name, default_value);
  value = node->get_parameter(name).get_value<T>();
}

void getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, float &value, const float &default_value)
{
  node->declare_parameter<double>(name, static_cast<double>(default_value));
  value = static_cast<float>(node->get_parameter(name).as_double());
}

void readParameters(const rclcpp::Node::SharedPtr &node)
{
  p_pre.reset(new Preprocess());
  getParam<bool>(node, "prop_at_freq_of_imu", prop_at_freq_of_imu, true);
  getParam<bool>(node, "use_imu_as_input", use_imu_as_input, true);
  getParam<bool>(node, "check_satu", check_satu, true);
  getParam<int>(node, "init_map_size", init_map_size, 100);
  getParam<bool>(node, "space_down_sample", space_down_sample, true);
  getParam<double>(node, "mapping.satu_acc", satu_acc, 3.0);
  getParam<double>(node, "mapping.satu_gyro", satu_gyro, 35.0);
  getParam<double>(node, "mapping.acc_norm", acc_norm, 1.0);
  getParam(node, "mapping.plane_thr", plane_thr, 0.05f);
  getParam<int>(node, "point_filter_num", p_pre->point_filter_num, 2);
  getParam<std::string>(node, "common.lid_topic", lid_topic, "/livox/lidar");
  getParam<std::string>(node, "common.imu_topic", imu_topic, "/livox/imu");
  getParam<bool>(node, "common.con_frame", con_frame, false);
  getParam<int>(node, "common.con_frame_num", con_frame_num, 1);
  getParam<bool>(node, "common.cut_frame", cut_frame, false);
  getParam<double>(node, "common.cut_frame_time_interval", cut_frame_time_interval, 0.1);
  getParam<double>(node, "common.time_lag_imu_to_lidar", time_lag_imu_to_lidar, 0.0);
  getParam<double>(node, "filter_size_surf", filter_size_surf_min, 0.5);
  getParam<double>(node, "filter_size_map", filter_size_map_min, 0.5);
  getParam<double>(node, "cube_side_length", cube_len, 200);
  getParam(node, "mapping.det_range", DET_RANGE, 300.f);
  getParam<double>(node, "mapping.fov_degree", fov_deg, 180);
  getParam<bool>(node, "mapping.imu_en", imu_en, true);
  getParam<bool>(node, "mapping.start_in_aggressive_motion", non_station_start, false);
  getParam<bool>(node, "mapping.extrinsic_est_en", extrinsic_est_en, true);
  getParam<double>(node, "mapping.imu_time_inte", imu_time_inte, 0.005);
  getParam<double>(node, "mapping.lidar_meas_cov", laser_point_cov, 0.1);
  getParam<double>(node, "mapping.acc_cov_input", acc_cov_input, 0.1);
  getParam<double>(node, "mapping.vel_cov", vel_cov, 20);
  getParam<double>(node, "mapping.gyr_cov_input", gyr_cov_input, 0.1);
  getParam<double>(node, "mapping.gyr_cov_output", gyr_cov_output, 0.1);
  getParam<double>(node, "mapping.acc_cov_output", acc_cov_output, 0.1);
  getParam<double>(node, "mapping.b_gyr_cov", b_gyr_cov, 0.0001);
  getParam<double>(node, "mapping.b_acc_cov", b_acc_cov, 0.0001);
  getParam<double>(node, "mapping.imu_meas_acc_cov", imu_meas_acc_cov, 0.1);
  getParam<double>(node, "mapping.imu_meas_omg_cov", imu_meas_omg_cov, 0.1);
  getParam<double>(node, "preprocess.blind", p_pre->blind, 1.0);
  getParam<int>(node, "preprocess.lidar_type", lidar_type, 1);
  getParam<int>(node, "preprocess.scan_line", p_pre->N_SCANS, 16);
  getParam<int>(node, "preprocess.scan_rate", p_pre->SCAN_RATE, 10);
  getParam<int>(node, "preprocess.timestamp_unit", p_pre->time_unit, 1);
  getParam<double>(node, "mapping.match_s", match_s, 81);
  getParam<bool>(node, "mapping.gravity_align", gravity_align, true);
  getParam<std::vector<double>>(node, "mapping.gravity", gravity, std::vector<double>());
  getParam<std::vector<double>>(node, "mapping.gravity_init", gravity_init, std::vector<double>());
  getParam<std::vector<double>>(node, "mapping.extrinsic_T", extrinT, std::vector<double>());
  getParam<std::vector<double>>(node, "mapping.extrinsic_R", extrinR, std::vector<double>());
  getParam<bool>(node, "odometry.publish_odometry_without_downsample", publish_odometry_without_downsample, false);
  getParam<bool>(node, "publish.path_en", path_en, true);
  getParam<bool>(node, "publish.scan_publish_en", scan_pub_en, true);
  getParam<bool>(node, "publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
  getParam<bool>(node, "runtime_pos_log_enable", runtime_pos_log, false);
  getParam<bool>(node, "pcd_save.pcd_save_en", pcd_save_en, false);
  getParam<int>(node, "pcd_save.interval", pcd_save_interval, -1);
}
