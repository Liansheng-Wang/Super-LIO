/**
 * @file alias.h
 * @author Liansheng Wang (lswang@mail.ecust.edu.cn)
 * @date 2023-10-06
 * @copyright Copyright (c) 2023
 */

 #ifndef ALIAS_TYPES_H_H_
 #define ALIAS_TYPES_H_H_
 
 #include <vector>
 #include <Eigen/Core>
 #include <Eigen/Dense>
 #include <Eigen/Geometry>
 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 
 namespace velodyne_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D;
     float intensity;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace velodyne_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, time, time))
 
 
 namespace NCLT {
 struct EIGEN_ALIGN16 Point {
     float x;
     float y;
     float z;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace velodyne_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(NCLT::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, time, time))
 
 
 namespace ouster_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D;
     float intensity;
     uint32_t t;
     uint32_t range;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace ouster_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint32_t, t, t)
                                   (std::uint32_t, range, range)
                                   )
 
 
 namespace hesai_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D
     float intensity;
     double timestamp;
     uint16_t ring; 
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (std::uint16_t, ring, ring)
                                   )
 
 
namespace LI2Sup {
struct EIGEN_ALIGN16 PointXTZIT {
  PCL_ADD_POINT4D
  float intensity;
  double offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXTZIT() = default;

  PointXTZIT(float x_, float y_, float z_, float intensity_, double time_)
  {
    x = x_;
    y = y_;
    z = z_;
    intensity = intensity_;
    offset_time = time_;
  }
};
}
 
POINT_CLOUD_REGISTER_POINT_STRUCT(LI2Sup::PointXTZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, offset_time, offset_time)
                                  )
 
 
 
namespace LivoxSIM {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
 
 
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxSIM::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  )

// x, y, z, intensity, tag, line, timestamp
 
namespace BASIC {

// using scalar = double;
using scalar = float;

/* alias for eigen */
using V2 = Eigen::Matrix<scalar, 2, 1>;
using V3 = Eigen::Matrix<scalar, 3, 1>;
using V4 = Eigen::Matrix<scalar, 4, 1>;
using V5 = Eigen::Matrix<scalar, 5, 1>;
using V6 = Eigen::Matrix<scalar, 6, 1>;
using V7 = Eigen::Matrix<scalar, 7, 1>;
using V8 = Eigen::Matrix<scalar, 8, 1>;
using V9 = Eigen::Matrix<scalar, 9, 1>;
using V12 = Eigen::Matrix<scalar, 12, 1>;
using V15 = Eigen::Matrix<scalar, 15, 1>;
using V18 = Eigen::Matrix<scalar, 18, 1>;
using V24 = Eigen::Matrix<scalar, 24, 1>;
using V30 = Eigen::Matrix<scalar, 30, 1>;
using V36 = Eigen::Matrix<scalar, 36, 1>;
using V40 = Eigen::Matrix<scalar, 40, 1>;
using VX  = Eigen::Matrix<scalar, -1, 1>;

using VV3 = std::vector<V3, Eigen::aligned_allocator<V3>>;
using VV4 = std::vector<V4, Eigen::aligned_allocator<V4>>;
using VV5 = std::vector<V5, Eigen::aligned_allocator<V5>>;

using M1 = Eigen::Matrix<scalar, 1, 1>;
using M2 = Eigen::Matrix<scalar, 2, 2>;
using M3 = Eigen::Matrix<scalar, 3, 3>;
using M4 = Eigen::Matrix<scalar, 4, 4>;
using M5 = Eigen::Matrix<scalar, 5, 5>;
using M6 = Eigen::Matrix<scalar, 6, 6>;
using M7 = Eigen::Matrix<scalar, 7, 7>;
using M8 = Eigen::Matrix<scalar, 8, 8>;
using M9 = Eigen::Matrix<scalar, 9, 9>;
using M12 = Eigen::Matrix<scalar, 12, 12>;
using M15 = Eigen::Matrix<scalar, 15, 15>;
using M18 = Eigen::Matrix<scalar, 18, 18>;
using M24 = Eigen::Matrix<scalar, 24, 24>;

using M3_2 = Eigen::Matrix<scalar, 3, 2>;
using M2_3 = Eigen::Matrix<scalar, 2, 3>;
using M6_7 = Eigen::Matrix<scalar, 6, 7>;

using MX3 = Eigen::Matrix<scalar, Eigen::Dynamic, 3>;

using Quat = Eigen::Quaternion<scalar>;

///////////////////////////////////////////////////////////
/* alias for eigen */
using V2f = Eigen::Matrix<float, 2, 1>;
using V3f = Eigen::Matrix<float, 3, 1>;
using V4f = Eigen::Matrix<float, 4, 1>;
using V5f = Eigen::Matrix<float, 5, 1>;
using V6f = Eigen::Matrix<float, 6, 1>;
using V7f = Eigen::Matrix<float, 7, 1>;
using V8f = Eigen::Matrix<float, 8, 1>;
using V9f = Eigen::Matrix<float, 9, 1>;
using V12f = Eigen::Matrix<float, 12, 1>;
using V15f = Eigen::Matrix<float, 15, 1>;
using V18f = Eigen::Matrix<float, 18, 1>;
using V24f = Eigen::Matrix<float, 24, 1>;
using V30f = Eigen::Matrix<float, 30, 1>;
using V36f = Eigen::Matrix<float, 36, 1>;
using V40f = Eigen::Matrix<float, 40, 1>;
using VXf  = Eigen::Matrix<float, -1, 1>;

using VV3f = std::vector<V3f, Eigen::aligned_allocator<V3f>>;
using VV4f = std::vector<V4f, Eigen::aligned_allocator<V4f>>;
using VV5f = std::vector<V5f, Eigen::aligned_allocator<V5f>>;

using M1f = Eigen::Matrix<float, 1, 1>;
using M2f = Eigen::Matrix<float, 2, 2>;
using M3f = Eigen::Matrix<float, 3, 3>;
using M4f = Eigen::Matrix<float, 4, 4>;
using M5f = Eigen::Matrix<float, 5, 5>;
using M6f = Eigen::Matrix<float, 6, 6>;
using M7f = Eigen::Matrix<float, 7, 7>;
using M8f = Eigen::Matrix<float, 8, 8>;
using M9f = Eigen::Matrix<float, 9, 9>;
using M12f = Eigen::Matrix<float, 12, 12>;
using M15f = Eigen::Matrix<float, 15, 15>;
using M18f = Eigen::Matrix<float, 18, 18>;
using M24f = Eigen::Matrix<float, 24, 24>;

using M3_2f = Eigen::Matrix<float, 3, 2>;
using M2_3f = Eigen::Matrix<float, 2, 3>;

using MX3f = Eigen::Matrix<float, Eigen::Dynamic, 3>;

using Quatf = Eigen::Quaternion<float>;

///////////////////////////////////////////////////////////
using V2d = Eigen::Matrix<double, 2, 1>;
using V3d = Eigen::Matrix<double, 3, 1>;
using V4d = Eigen::Matrix<double, 4, 1>;
using V5d = Eigen::Matrix<double, 5, 1>;
using V6d = Eigen::Matrix<double, 6, 1>;
using V7d = Eigen::Matrix<double, 7, 1>;
using V8d = Eigen::Matrix<double, 8, 1>;
using V9d = Eigen::Matrix<double, 9, 1>;
using V12d = Eigen::Matrix<double, 12, 1>;
using V15d = Eigen::Matrix<double, 15, 1>;
using V18d = Eigen::Matrix<double, 18, 1>;
using V24d = Eigen::Matrix<double, 24, 1>;
using V30d = Eigen::Matrix<double, 30, 1>;
using V36d = Eigen::Matrix<double, 36, 1>;
using V40d = Eigen::Matrix<double, 40, 1>;
using VXd  = Eigen::Matrix<double, -1, 1>;

using VV3d = std::vector<V3d, Eigen::aligned_allocator<V3d>>;
using VV4d = std::vector<V4d, Eigen::aligned_allocator<V4d>>;
using VV5d = std::vector<V5d, Eigen::aligned_allocator<V5d>>;

using M1d = Eigen::Matrix<double, 1, 1>;
using M2d = Eigen::Matrix<double, 2, 2>;
using M3d = Eigen::Matrix<double, 3, 3>;
using M4d = Eigen::Matrix<double, 4, 4>;
using M5d = Eigen::Matrix<double, 5, 5>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M7d = Eigen::Matrix<double, 7, 7>;
using M8d = Eigen::Matrix<double, 8, 8>;
using M9d = Eigen::Matrix<double, 9, 9>;
using M12d = Eigen::Matrix<double, 12, 12>;
using M15d = Eigen::Matrix<double, 15, 15>;
using M18d = Eigen::Matrix<double, 18, 18>;
using M24d = Eigen::Matrix<double, 24, 24>;

using M3_2d = Eigen::Matrix<double, 3, 2>;
using M2_3d = Eigen::Matrix<double, 2, 3>;

using MX3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;

using Quatd = Eigen::Quaternion<double>;


///////////////////////////////////////////////////////////
const M3 Eye3 = M3::Identity();
const V3 ZeroV3(0, 0, 0);
const V3 EyeV3(1,1,1);

using V2i = Eigen::Vector2i;
using V3i = Eigen::Vector3i;
using VXi = Eigen::VectorXi;
using VV3i = std::vector<V3i, Eigen::aligned_allocator<V3i>>;


/* pcl: using type */
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;


/* eigen point_cloud: using type */
const M4 cov4 = M4::Identity() * 0.1;
struct EIGEN_ALIGN32 PointCov {
  V4 point = V4::Zero();
  M4 cov = cov4;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
 
using PointsCov = std::vector<PointCov, Eigen::aligned_allocator<PointCov>>;

}
#endif
 