#pragma once

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <iostream>
#include <cmath>

struct LandMarkObservation
{
	void CoutInf()
	{
		std::cout.flags(std::ios::fixed);
		std::cout.precision(6);
		std::cout << "ObservationInf: " << uiLiDARFrameOffset << " " << LandMarkID <<
			" " << Observation.transpose() << std::endl;
	}
	int uiLiDARFrameOffset;
	int LandMarkID;
	Eigen::Vector2d Observation;
};

struct LandMarkObservationEdge
{
	void CoutInf()
	{
		std::cout.flags(std::ios::fixed);
		std::cout.precision(6);
		std::cout << "TargetFrameInf:" << " " << nTargetFrameOffset << " " << TargetObs.transpose() << std::endl;
		std::cout << "SourceFrameInf:" << " " << nSourceFrameOffset << " " << SourceObs.transpose() << std::endl;
		std::cout << "LandMarkID:" << " " << nLandMarkID << std::endl;
	}
	int nTargetFrameOffset;
	int nSourceFrameOffset;
	Eigen::Vector2d TargetObs;
	Eigen::Vector2d SourceObs;
	int nLandMarkID;
};

//Normalize the angle
double GN_NormalizationAngleNIE(double angle);

//Transform Pose to TransMatrix
Eigen::Matrix3d PoseToTransNIE(Eigen::Vector3d pose);

//Transform Trans to Pose
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans);

class Pose3dVertex:public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual void setToOriginImpl()
	{
		_estimate << 0.0, 0.0, 0.0;
	}

	virtual void oplusImpl(const double* update)
	{
		_estimate += Eigen::Vector3d(update);
		_estimate(2) = GN_NormalizationAngleNIE(_estimate(2));
	}
	
	virtual bool read(std::istream& in) { return true; };
	virtual bool write(std::ostream& out)const { return true; };
};

class ObservePointEdge:public g2o::BaseBinaryEdge<2,Eigen::Vector2d,Pose3dVertex,Pose3dVertex>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ObservePointEdge(Eigen::Vector2d _v2dObserve1):BaseBinaryEdge(), 
		m_v2dObserve1(_v2dObserve1)
	{
	}

	void computeError()
	{
		const Pose3dVertex* v0 = static_cast<const Pose3dVertex*> (_vertices[0]);
		const Pose3dVertex* v1 = static_cast<const Pose3dVertex*> (_vertices[1]);
		Eigen::Matrix3d Pose1Matrix = PoseToTransNIE(Eigen::Vector3d(v0->estimate()));
		Eigen::Matrix3d Pose2Matrix = PoseToTransNIE(Eigen::Vector3d(v1->estimate()));
		Eigen::Matrix3d Pose2MatrixInverse = Pose2Matrix.inverse();
		
		Eigen::Vector3d v3dObserve1(m_v2dObserve1(0), m_v2dObserve1(1), 1.0);
		_error = _measurement - (Pose2MatrixInverse * Pose1Matrix * v3dObserve1).head<2>();
	}

	virtual bool read(std::istream& in) { return true; };
	virtual bool write(std::ostream& out)const { return true; };
private:
	Eigen::Vector2d m_v2dObserve1;
};



//G2O optimizer set
void G2O_Optimizer(const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess,
	const std::vector<LandMarkObservationEdge>& vLandMarkObservationEdges,
	std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose);

//Transform Pointw to PointObs
Eigen::Vector2d TransformPoint(Eigen::Vector3d v3dPose, Eigen::Vector2d Point);

//Generate the LandMark
void GenerateLandMark(std::vector<Eigen::Vector2d>& vv2dLandMarkLib);

//Generate the LiDARPose real/guess
void GenerateLiDARPose(std::vector<Eigen::Vector3d>& vv3dLiDARRealPose, std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess);

//Generate the observation 
void GenerateObservations(const std::vector<Eigen::Vector2d>& vv2dLandMarkLib,
	const std::vector<Eigen::Vector3d>& vv3dLiDARRealPose,
	std::vector<LandMarkObservation>& vLandMarkObservations);

//Construct the Observation Edge
void ConstructObserEdge(const std::vector<LandMarkObservation>& vLandMarkObservations,
	std::vector<LandMarkObservationEdge>& vLandMarkObservationEdges);

//Analyze the pose error
void AnalyzeError(const std::vector<Eigen::Vector3d>& vv3dLiDARRealPose,
	const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess,
	const std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose);
