#pragma once
#include "G2OOptimizer.h"

struct Fra2FraTransEdge
{
	Fra2FraTransEdge(int _ntargetFrameOffset, int _nsourceFrameOffset, Eigen::Vector3d _transPose)
	{
		ntargetFrameOffset = _ntargetFrameOffset;
		nsourceFrameOffset = _nsourceFrameOffset;
		transPose = _transPose;
	}
	void Coutinf()
	{
		std::cout.flags(std::ios::fixed);
		std::cout.precision(6);
		std::cout << "targetFrameOffset: " << ntargetFrameOffset << "\t"
			<< "sourceFrameOffset: " << nsourceFrameOffset << "\t"
			<< "transPose: " << transPose.transpose() << std::endl;

	}
	int ntargetFrameOffset;
	int nsourceFrameOffset;
	Eigen::Vector3d transPose;
};

class Pose3dEdge :public g2o::BaseBinaryEdge<3, Eigen::Vector3d, Pose3dVertex, Pose3dVertex>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Pose3dEdge() {};

	void computeError()
	{
		const Pose3dVertex* v0 = static_cast<const Pose3dVertex*> (_vertices[0]);
		const Pose3dVertex* v1 = static_cast<const Pose3dVertex*> (_vertices[1]);

		Eigen::Matrix3d Pose0 = PoseToTransNIE(v0->estimate());
		Eigen::Matrix3d Pose1 = PoseToTransNIE(v1->estimate());
		Eigen::Vector3d transPose = TransToPose(Pose0.inverse()* Pose1);
		_error = _measurement - transPose;
	}

	virtual bool read(std::istream& in) { return true; };
	virtual bool write(std::ostream& out)const { return true; };
};

//G2O optimizer set
void G2O_OptimizerFra2Fra(const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess,
	const std::vector<Fra2FraTransEdge>& vFra2FraTransEdges,
	std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose);