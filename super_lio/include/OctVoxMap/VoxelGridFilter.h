
#ifndef VOXEL_GRID_CLOSEST_H
#define VOXEL_GRID_CLOSEST_H


#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "tsl/robin_hood.h"


namespace LI2Sup {


template<typename PointType>
class VoxelGridClosest {
private:
  using Point = PointType;
  using PointCloud = pcl::PointCloud<Point>;
  using CloudPtr = typename PointCloud::Ptr;

  CloudPtr cloud_;
  float voxel_size_ = 0.5f;
  float inv_voxel_size_ = 2.0f;
  robin_hood::unordered_flat_map<std::size_t, std::size_t> voxel_map_;

  std::vector<Point, Eigen::aligned_allocator<Point>> points_;
  std::vector<float> dist2_;
  const Eigen::Vector3i offset_ = Eigen::Vector3i(1000, 1000, 1000);

public:
  VoxelGridClosest() {
    dist2_.reserve(10000);
    points_.reserve(10000);
    voxel_map_.reserve(10000);
  }

  void setLeafSize(float lx) {
    voxel_size_ = lx;
    inv_voxel_size_ = 1.0f / lx;
  }

  void setInputCloud(const CloudPtr& cloud) {
    cloud_ = cloud;
  }

  void filter(CloudPtr& output) {
    voxel_map_.clear();
    dist2_.clear();
    points_.clear();

    for (const auto& pt : cloud_->points) {
      Eigen::Vector3f pf = pt.getVector3fMap();
      Eigen::Vector3i idx = (pf * inv_voxel_size_).array().round().cast<int>();
      Eigen::Vector3f center = voxel_size_ * idx.cast<float>();
      float d2 = (pf - center).squaredNorm();

      idx += offset_; // Avoid negative indices
      const std::size_t key = ((std::size_t(idx[2])) << 30) | 
                              ((std::size_t(idx[1])) << 15) | 
                              ( std::size_t(idx[0]));

      auto it = voxel_map_.find(key);
      if (it == voxel_map_.end()) {
        voxel_map_.emplace(key, points_.size());
        points_.push_back(pt);
        dist2_.push_back(d2);
      } else if (d2 < dist2_[it->second]) {
        points_[it->second] = pt;
        dist2_[it->second] = d2;
      }
    }

    output->points.swap(points_);
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    output->header = cloud_->header;
  }
};

}
#endif // VOXEL_GRID_CLOSEST_H