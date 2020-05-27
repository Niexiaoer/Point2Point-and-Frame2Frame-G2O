#pragma once
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "G2OOptimizerICP.h"
#include "G2OOptimizer.h"

Eigen::Vector3d GetRigidTransform(const std::vector<CorrPoint>& vCorrPoints);