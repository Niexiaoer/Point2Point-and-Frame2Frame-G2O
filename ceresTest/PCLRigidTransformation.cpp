#include "PCLRigidTransformation.h"

Eigen::Vector3d GetRigidTransform(const std::vector<CorrPoint>& vCorrPoints)
{
	int nSize = vCorrPoints.size();
	pcl::PointCloud<pcl::PointXYZ>  m_TargetCloud;
	pcl::PointCloud<pcl::PointXYZ>  m_SourceCloud;

	m_TargetCloud.width = nSize;
	m_TargetCloud.height = 1;
	m_TargetCloud.is_dense = false;
	m_TargetCloud.resize(m_TargetCloud.width * m_TargetCloud.height);

	m_SourceCloud.width = nSize;
	m_SourceCloud.height = 1;
	m_SourceCloud.is_dense = false;
	m_SourceCloud.resize(m_SourceCloud.width * m_SourceCloud.height);

	for (int i = 0;i < nSize;i++)
	{
		m_TargetCloud.points[i].x = vCorrPoints.at(i).targetPoint(0);
		m_TargetCloud.points[i].y = vCorrPoints.at(i).targetPoint(1);
		m_TargetCloud.points[i].z = 0.0;

		m_SourceCloud.points[i].x = vCorrPoints.at(i).sourcePoint(0);
		m_SourceCloud.points[i].y = vCorrPoints.at(i).sourcePoint(1);
		m_SourceCloud.points[i].z = 0.0;
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
	TESVD.estimateRigidTransformation(m_SourceCloud,m_TargetCloud,  transformation2);

	Eigen::Matrix3d m3dRT = Eigen::Matrix3d::Identity();
	m3dRT.block<2,2>(0,0) = transformation2.block<2, 2>(0, 0).cast<double>();
	m3dRT.block<2, 1>(0, 2) = transformation2.block<2, 1>(0, 3).cast<double>();
	Eigen::Vector3d v3dRT = TransToPose(m3dRT);
    return v3dRT;
}
