/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LiDARFeature
{
  Nor,
  Poss_Plane,
  Real_Plane,  // 直线
  Edge_Jump,   // 边缘跳跃点
  Edge_Plane, // 直线边缘
  Wire,   // 线缆
  ZeroPoint
};
enum Surround
{
  Prev,  // 前一个点
  Next  // 后一个点
};
enum E_jump
{
  Nr_nor,
  Nr_zero, // 与发射方向夹角小于10度   入射角很大
  Nr_180,  // 与发射方向夹角大于170度  入射角很大
  Nr_inf,  // 超出探测范围
  Nr_blind // 在盲区
};

struct orgtype
{
  double range;  // XY平面投影点的range值
  double dista;  // 与下一个点的距离
  double angle[2]; // 当前点与前后两个点组成的两个向量，分别与当前点发射方向的余弦值
  double intersect; // 当前点与前后两个点组成的两个向量的点乘结果
  E_jump edj[2];
  LiDARFeature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float timestamp;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))
/*****************/

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  vector<orgtype> typess[128]; // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;
  
  // 盲区距离/盲区距离平方
  double blind, blind_sqr;
  bool feature_enabled, given_offset_time;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_full;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_surf;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_corn;

private:
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_