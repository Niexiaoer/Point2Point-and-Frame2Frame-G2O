
#include "G2OOptimizer.h"
#include <random>
#include <cmath>

#define LandMarkCellSize 10
#define LandMarkObsstd 0.03

double GN_NormalizationAngleNIE(double angle)
{
	if (angle > M_PI)
	{
		angle -= 2 * M_PI;
	}
	if (angle < -M_PI)
	{
		angle += 2 * M_PI;
	}
	return angle;
}

Eigen::Matrix3d PoseToTransNIE(Eigen::Vector3d pose)
{
	Eigen::Matrix3d TransformMatrix;
	TransformMatrix << cos(pose(2)), -sin(pose(2)), pose(0),
		sin(pose(2)), cos(pose(2)), pose(1),
		0.0, 0.0, 1.0;
	return TransformMatrix;
}

Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
	Eigen::Vector3d Pose;
	Pose << trans(0, 2), trans(1, 2), atan2(trans(1, 0), trans(0, 0));
	return Pose;
}

Eigen::Vector2d TransformPoint(Eigen::Vector3d v3dPose, Eigen::Vector2d Point)
{
	Eigen::Matrix3d TransformMatrix;
	TransformMatrix << cos(v3dPose(2)), -sin(v3dPose(2)), v3dPose(0),
		sin(v3dPose(2)), cos(v3dPose(2)), v3dPose(1),
		0.0, 0.0, 1.0;
	Eigen::Matrix3d TransformMatrixInverse = TransformMatrix.inverse();
	Eigen::Vector3d TransformedPoint;
	Eigen::Vector3d HomoPoint;
	HomoPoint << Point(0), Point(1), 1.0;
	TransformedPoint = TransformMatrixInverse * HomoPoint;
	return TransformedPoint.head<2>();
}

void GenerateLandMark(std::vector<Eigen::Vector2d>& vv2dLandMarkLib)
{
	Eigen::Vector2d v2dPos(0.0, 0.0);
	for (int i = 0;i < 3;i++)
	{
		v2dPos(1) = 0.0;
		v2dPos(0) += 1.0;
		for (int j = 0;j < 3;j++)
		{
			v2dPos(1) += 1.0;
			Eigen::Vector2d pose0(v2dPos(0) - 1.0, v2dPos(1) - 1.0);
			for (int k = 0;k < LandMarkCellSize;k++)
			{
				double x = rand() % 10000 / (double)10000 + pose0(0);
				double y = rand() % 10000 / (double)10000 + pose0(1);
				Eigen::Vector2d temp(x, y);
				vv2dLandMarkLib.push_back(temp);
			}
		}
	}

}

void GenerateLiDARPose(std::vector<Eigen::Vector3d>& vv3dLiDARRealPose, std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess)
{
	Eigen::Vector3d Pose1(1.0, 1.0, 0);
	Eigen::Vector3d Pose2(1.0, 2.0, 0.5 * M_PI);
	Eigen::Vector3d Pose3(2.0, 1.0, M_PI);
	Eigen::Vector3d Pose4(2.0, 2.0, -0.5 * M_PI);
	vv3dLiDARRealPose.push_back(Pose1);
	vv3dLiDARRealPose.push_back(Pose2);
	vv3dLiDARRealPose.push_back(Pose3);
	vv3dLiDARRealPose.push_back(Pose4);

	Eigen::Vector3d InitialPoseGuess1(1.0, 1.0, 0);
	Eigen::Vector3d InitialPoseGuess2(0.9, 2.1, 0.5 * M_PI + 0.1 * M_PI);
	Eigen::Vector3d InitialPoseGuess3(2.1, 1.1, M_PI - 0.1 * M_PI);
	Eigen::Vector3d InitialPoseGuess4(2.1, 1.9, -0.5 * M_PI - 0.1 * M_PI);
	vv3dLiDARInitialPoseGuess.push_back(InitialPoseGuess1);
	vv3dLiDARInitialPoseGuess.push_back(InitialPoseGuess2);
	vv3dLiDARInitialPoseGuess.push_back(InitialPoseGuess3);
	vv3dLiDARInitialPoseGuess.push_back(InitialPoseGuess4);

}

void GenerateObservations(const std::vector<Eigen::Vector2d>& vv2dLandMarkLib,
	const std::vector<Eigen::Vector3d>& vv3dLiDARRealPose,
	std::vector<LandMarkObservation>& vLandMarkObservations)
{
	std::default_random_engine rng;
	std::normal_distribution<double> distRnormal{ 0.0, LandMarkObsstd };
	for (int i = 0;i < vv3dLiDARRealPose.size();i++)
	{
		Eigen::Vector3d LiDARRealPoseTemp = vv3dLiDARRealPose.at(i);
		for (int j = 0;j < vv2dLandMarkLib.size();j++)
		{
			Eigen::Vector2d LandMarkTemp = vv2dLandMarkLib.at(j);
			if ((LiDARRealPoseTemp.head<2>() - LandMarkTemp).norm() < sqrt(2.0))
			{
				LandMarkObservation LMObsTmp;
				LMObsTmp.uiLiDARFrameOffset = i;
				LMObsTmp.LandMarkID = j;
				Eigen::Vector2d ObserTmp = TransformPoint(LiDARRealPoseTemp, LandMarkTemp);
				ObserTmp(0) += distRnormal(rng);
				ObserTmp(1) += distRnormal(rng);
				LMObsTmp.Observation = ObserTmp;
				vLandMarkObservations.push_back(LMObsTmp);
			}
		}
	}
}

void ConstructObserEdge(const std::vector<LandMarkObservation>& vLandMarkObservations,
	std::vector<LandMarkObservationEdge>& vLandMarkObservationEdges)
{
	for (int i = 0;i < vLandMarkObservations.size();i++)
	{
		LandMarkObservation LMObTmp1 = vLandMarkObservations.at(i);
		for (int j = i + 1;j < vLandMarkObservations.size();j++)
		{
			LandMarkObservation LMObTmp2 = vLandMarkObservations.at(j);
			if (LMObTmp1.LandMarkID == LMObTmp2.LandMarkID)
			{
				LandMarkObservationEdge EdgeTmp;
				EdgeTmp.nTargetFrameOffset = LMObTmp1.uiLiDARFrameOffset;
				EdgeTmp.nSourceFrameOffset = LMObTmp2.uiLiDARFrameOffset;
				EdgeTmp.nLandMarkID = LMObTmp1.LandMarkID;
				EdgeTmp.TargetObs = LMObTmp1.Observation;
				EdgeTmp.SourceObs = LMObTmp2.Observation;
				vLandMarkObservationEdges.push_back(EdgeTmp);
			}
		}
	}
}

void computeError(const std::vector<Eigen::Vector3d>& vv3dLiDARRealPose, 
	const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess,
	const std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose)
{
	std::cout << "Initial Error Analysis:" << std::endl;
	std::cout << "Posx:\tPosy:\tAtt(Deg):\t" << std::endl;
	for (int i = 0;i < vv3dLiDARRealPose.size();i++)
	{
		Eigen::Vector3d error = vv3dLiDARInitialPoseGuess.at(i) - vv3dLiDARRealPose.at(i);
		std::cout.flags(std::ios::fixed);
		std::cout.precision(6);
		std::cout << error(0) << "\t" << error(1) << "\t" << error(2) * 180.0 / M_PI << std::endl;
	}

	std::cout << "Optimized Error Analysis:" << std::endl;
	std::cout << "Posx:\tPosy:\tAtt(Deg):\t" << std::endl;
	for (int i =0;i< vv3dLiDARRealPose.size();i++)
	{
		Eigen::Vector3d error = vv3dLiDARFinalPose.at(i) - vv3dLiDARRealPose.at(i);
		std::cout.flags(std::ios::fixed);
		std::cout.precision(6);
		std::cout << error(0) << "\t" << error(1) << "\t" << error(2) * 180.0 / M_PI << std::endl;
	}
}

void G2O_Optimizer(const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess,
	const std::vector<LandMarkObservationEdge>& vLandMarkObservationEdges,
	std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose)
{
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> Block32;
	Block32::LinearSolverType* linearSolver(new g2o::LinearSolverCSparse<Block32::PoseMatrixType>());
	// create the block solver on the top of the linear solver
	Block32* blockSolver(new Block32(linearSolver));

	//create the algorithm to carry out the optimization
	//g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
	g2o::OptimizationAlgorithmDogleg* optimizationAlgorithm = new g2o::OptimizationAlgorithmDogleg(blockSolver);
	//g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
	g2o::SparseOptimizer optimizer;

	optimizer.setAlgorithm(optimizationAlgorithm);
	optimizer.setVerbose(false);

	//Add Vertex
	std::vector<Pose3dVertex*> vPose3dVertexs;
	for (int i = 0;i < vv3dLiDARInitialPoseGuess.size();i++)
	{
		Pose3dVertex* v = new Pose3dVertex();
		v->setEstimate(vv3dLiDARInitialPoseGuess.at(i));
		v->setId(i);
		if (i == 0)
		{
			v->setFixed(true);
		}
		optimizer.addVertex(v);
		vPose3dVertexs.push_back(v);
	}

	//Add Edge
	int nedgeId = 0;
	for (int i = 0;i < vLandMarkObservationEdges.size();i++)
	{
		LandMarkObservationEdge edgeTmp = vLandMarkObservationEdges.at(i);
		ObservePointEdge* edge = new ObservePointEdge(edgeTmp.TargetObs);
		edge->setId(i);
		edge->setVertex(0, vPose3dVertexs.at(edgeTmp.nTargetFrameOffset));
		edge->setVertex(1, vPose3dVertexs.at(edgeTmp.nSourceFrameOffset));
		edge->setMeasurement(edgeTmp.SourceObs);
		Eigen::Matrix2d InfoMatrix = Eigen::Matrix2d::Identity();
		InfoMatrix *= 10.0;
		edge->setInformation(InfoMatrix);
		optimizer.addEdge(edge);
	}
	optimizer.initializeOptimization();
	optimizer.optimize(1000);
	std::cout << "g2o optimizer Finished!" << std::endl;

	for (int i = 0;i < vPose3dVertexs.size();i++)
	{
		vv3dLiDARFinalPose.push_back(vPose3dVertexs.at(i)->estimate());
	}
}