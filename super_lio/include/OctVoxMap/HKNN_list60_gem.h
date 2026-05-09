#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>



/// The last number is useless
static const std::array<float, 6> orders_min_dis = {
  0.250000, 0.353553, 0.500000, 0.559017, 0.612372, 10 
};
static const std::array<float, 6> orders_min_dis2 = {
  0.062500, 0.125000, 0.250000, 0.312500, 0.375000, 100
};


alignas(16) static const std::array<Eigen::Vector3i, 60> HKNN_neighbor_voxel = 
{
  Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(-1, 0, -1), Eigen::Vector3i(0, -1, -1), 
  Eigen::Vector3i(-1, -1, 0), Eigen::Vector3i(-1, -1, -1), Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, -1, 1), 
  Eigen::Vector3i(1, -1, 0), Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(0, 1, -1), Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, -1, 1), 
  Eigen::Vector3i(-1, 1, -1), Eigen::Vector3i(1, -1, -1), Eigen::Vector3i(0, 0, -2), Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, 0, 1), 
  Eigen::Vector3i(-2, 0, 0), Eigen::Vector3i(0, -2, 0), Eigen::Vector3i(-2, 0, -1), Eigen::Vector3i(-1, 0, -2), Eigen::Vector3i(-1, -2, 0), Eigen::Vector3i(-2, -1, 0), 
  Eigen::Vector3i(0, -2, -1), Eigen::Vector3i(1, 1, -1), Eigen::Vector3i(-1, 1, 1), Eigen::Vector3i(1, -1, 1), Eigen::Vector3i(0, -1, -2), Eigen::Vector3i(-2, -1, -1), 
  Eigen::Vector3i(-1, -1, -2), Eigen::Vector3i(-1, -2, -1), Eigen::Vector3i(-2, 1, 0), Eigen::Vector3i(1, -2, 0), Eigen::Vector3i(0, -2, 1), Eigen::Vector3i(0, 1, -2), 
  Eigen::Vector3i(1, 0, -2), Eigen::Vector3i(1, 1, 1), Eigen::Vector3i(-2, 0, 1), Eigen::Vector3i(1, -1, -2), Eigen::Vector3i(1, -2, -1), Eigen::Vector3i(-2, 1, -1), 
  Eigen::Vector3i(-2, -1, 1), Eigen::Vector3i(-1, 1, -2), Eigen::Vector3i(-1, -2, 1), Eigen::Vector3i(0, -2, -2), Eigen::Vector3i(-2, 0, -2), Eigen::Vector3i(-2, 1, 1), 
  Eigen::Vector3i(1, 1, -2), Eigen::Vector3i(1, -2, 1), Eigen::Vector3i(-2, -2, 0), Eigen::Vector3i(-2, -1, -2), Eigen::Vector3i(-2, -2, -1), Eigen::Vector3i(-1, -2, -2), 
  };

static constexpr std::array<uint16_t, 7> flat_search_order_offsets = {{
  0, 43, 135, 219, 321, 465, 593,
}};

static constexpr std::array<uint8_t, 593> flat_search_order = {{
  // Group0
  0, 8, 0, 1, 2, 3, 4, 5, 6, 7, 
  1, 4, 2, 3, 6, 7, 
  2, 4, 4, 5, 6, 7, 
  3, 4, 1, 3, 5, 7, 
  4, 2, 5, 7, 
  5, 2, 6, 7, 
  6, 2, 3, 7, 
  7, 1, 7, 

  // Group1
  1, 4, 0, 1, 4, 5, 
  2, 4, 0, 1, 2, 3, 
  3, 4, 0, 2, 4, 6, 
  4, 4, 1, 3, 4, 6, 
  5, 4, 2, 3, 4, 5, 
  6, 4, 1, 2, 5, 6, 
  7, 3, 3, 5, 6, 
  8, 4, 0, 2, 4, 6, 
  9, 4, 0, 1, 4, 5, 
  10, 4, 0, 1, 2, 3, 
  11, 2, 2, 3, 
  12, 2, 2, 6, 
  13, 2, 4, 6, 
  14, 2, 4, 5, 
  15, 2, 1, 5, 
  16, 2, 1, 3, 
  17, 1, 3, 
  18, 1, 5, 
  19, 1, 6, 

  // Group2
  4, 2, 0, 2, 
  5, 2, 0, 1, 
  6, 2, 0, 4, 
  7, 4, 0, 1, 2, 4, 
  11, 2, 0, 1, 
  12, 2, 0, 4, 
  13, 2, 0, 2, 
  14, 2, 0, 1, 
  15, 2, 0, 4, 
  16, 2, 0, 2, 
  17, 3, 0, 1, 2, 
  18, 3, 0, 1, 4, 
  19, 3, 0, 2, 4, 
  21, 2, 0, 1, 
  22, 2, 0, 4, 
  23, 2, 0, 2, 
  31, 2, 0, 4, 
  32, 2, 0, 1, 
  33, 2, 0, 2, 
  43, 1, 0, 

  // Group3
  8, 4, 1, 3, 5, 7, 
  9, 4, 2, 3, 6, 7, 
  10, 4, 4, 5, 6, 7, 
  11, 2, 6, 7, 
  12, 2, 3, 7, 
  13, 2, 5, 7, 
  14, 2, 6, 7, 
  15, 2, 3, 7, 
  16, 2, 5, 7, 
  17, 1, 7, 
  18, 1, 7, 
  19, 1, 7, 
  20, 4, 4, 5, 6, 7, 
  24, 4, 1, 3, 5, 7, 
  25, 4, 2, 3, 6, 7, 
  26, 2, 5, 7, 
  27, 2, 5, 7, 
  28, 2, 3, 7, 
  29, 2, 3, 7, 
  30, 2, 6, 7, 
  34, 2, 6, 7, 
  35, 1, 7, 
  36, 1, 7, 
  37, 1, 7, 

  // Group4
  11, 2, 4, 5, 
  12, 2, 1, 5, 
  13, 2, 1, 3, 
  14, 2, 2, 3, 
  15, 2, 2, 6, 
  16, 2, 4, 6, 
  17, 2, 5, 6, 
  18, 2, 3, 6, 
  19, 2, 3, 5, 
  21, 4, 2, 3, 4, 5, 
  22, 4, 1, 2, 5, 6, 
  23, 4, 1, 3, 4, 6, 
  26, 2, 1, 3, 
  27, 2, 4, 6, 
  28, 2, 2, 6, 
  29, 2, 1, 5, 
  30, 2, 2, 3, 
  31, 2, 5, 6, 
  32, 2, 3, 5, 
  33, 2, 3, 6, 
  34, 2, 4, 5, 
  35, 2, 3, 5, 
  36, 2, 5, 6, 
  37, 2, 3, 6, 
  38, 2, 1, 5, 
  39, 2, 2, 6, 
  40, 2, 2, 3, 
  41, 2, 4, 5, 
  42, 2, 4, 6, 
  44, 2, 1, 3, 
  45, 1, 6, 
  46, 1, 6, 
  47, 1, 5, 
  48, 1, 3, 
  49, 1, 5, 
  50, 1, 3, 

  // Group5
  17, 1, 4, 
  18, 1, 2, 
  19, 1, 1, 
  21, 2, 6, 7, 
  22, 2, 3, 7, 
  23, 2, 5, 7, 
  31, 3, 1, 2, 7, 
  32, 3, 2, 4, 7, 
  33, 3, 1, 4, 7, 
  35, 1, 1, 
  36, 1, 4, 
  37, 1, 2, 
  38, 2, 3, 7, 
  39, 2, 3, 7, 
  40, 2, 6, 7, 
  41, 2, 6, 7, 
  42, 2, 5, 7, 
  43, 3, 1, 2, 4, 
  44, 2, 5, 7, 
  45, 2, 4, 7, 
  46, 2, 2, 7, 
  47, 2, 1, 7, 
  48, 2, 1, 7, 
  49, 2, 4, 7, 
  50, 2, 2, 7, 
  51, 2, 6, 7, 
  52, 2, 5, 7, 
  53, 1, 1, 
  54, 1, 4, 
  55, 1, 2, 
  56, 2, 3, 7, 
  57, 1, 7, 
  58, 1, 7, 
  59, 1, 7, 

}};



