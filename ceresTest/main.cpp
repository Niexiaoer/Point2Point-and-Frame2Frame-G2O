#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "G2OOptimizerICP.h"
#include "G2OOptimizer.h"
#include "G2OOptimizerFra2Fra.h"
#include "PCLRigidTransformation.h"


int main()
{
	std::cout << "Main function" << std::endl;

	std::vector<Eigen::Vector2d> vv2dLandMarkLib;
	GenerateLandMark(vv2dLandMarkLib);

	std::vector<Eigen::Vector3d> vv3dLiDARRealPose;
	std::vector<Eigen::Vector3d> vv3dLiDARInitialPoseGuess;
	GenerateLiDARPose(vv3dLiDARRealPose,vv3dLiDARInitialPoseGuess);

	std::vector<LandMarkObservation> vLandMarkObservations;
	GenerateObservations(vv2dLandMarkLib, vv3dLiDARRealPose, vLandMarkObservations);
	//for (int i =0;i< vLandMarkObservations.size();i++)
	//{
	//	vLandMarkObservations.at(i).CoutInf();
	//	std::cout << "LandMarkPos: " << vv2dLandMarkLib.at(vLandMarkObservations.at(i).LandMarkID).transpose() << std::endl << std::endl;
	//}

	std::vector<LandMarkObservationEdge> vLandMarkObservationEdges;
	ConstructObserEdge(vLandMarkObservations, vLandMarkObservationEdges);
	//for (int i = 0;i < vLandMarkObservationEdges.size();i++)
	//{
	//	vLandMarkObservationEdges.at(i).CoutInf();
	//	std::cout << std::endl;
	//}

	////////////Point to Point Edge G2O///////////////////
	std::cout << "Point to Point Edge G2O Test!\n";
	std::vector<Eigen::Vector3d> vv3dLiDARFinalPose1;
	G2O_Optimizer(vv3dLiDARInitialPoseGuess, vLandMarkObservationEdges, vv3dLiDARFinalPose1);
	AnalyzeError(vv3dLiDARRealPose, vv3dLiDARInitialPoseGuess, vv3dLiDARFinalPose1);
	////////////Point to Point Edge G2O///////////////////

	std::cout << std::endl;
	//////////////Frame to Frame Edge G2O//////////////////
	std::cout << "Frame to Frame Edge G2O Test!\n";
	std::vector<CorrPoint> vCorrPoints01;
	std::vector<CorrPoint> vCorrPoints02;
	std::vector<CorrPoint> vCorrPoints03;
	std::vector<CorrPoint> vCorrPoints12;
	std::vector<CorrPoint> vCorrPoints13;
	std::vector<CorrPoint> vCorrPoints23;
	for (int i = 0;i< vLandMarkObservationEdges.size();i++)
	{
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 0&&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 1)
		{
			vCorrPoints01.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 0 &&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 2)
		{
			vCorrPoints02.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 0 &&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 3)
		{
			vCorrPoints03.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 1 &&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 2)
		{
			vCorrPoints12.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 1 &&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 3)
		{
			vCorrPoints13.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}
		if (vLandMarkObservationEdges.at(i).nTargetFrameOffset == 2 &&
			vLandMarkObservationEdges.at(i).nSourceFrameOffset == 3)
		{
			vCorrPoints23.push_back(CorrPoint(vLandMarkObservationEdges.at(i).TargetObs, vLandMarkObservationEdges.at(i).SourceObs));
		}

	}
	Eigen::Vector3d transFinal01;
	Eigen::Vector3d transFinal02;
	Eigen::Vector3d transFinal03;
	Eigen::Vector3d transFinal12;
	Eigen::Vector3d transFinal13;
	Eigen::Vector3d transFinal23;

	//Eigen::Matrix3d transGuess0 = PoseToTransNIE(vv3dLiDARInitialPoseGuess.at(0));
	//Eigen::Matrix3d transGuess1 = PoseToTransNIE(vv3dLiDARInitialPoseGuess.at(1));
	//Eigen::Matrix3d transGuess2 = PoseToTransNIE(vv3dLiDARInitialPoseGuess.at(2));
	//Eigen::Matrix3d transGuess3 = PoseToTransNIE(vv3dLiDARInitialPoseGuess.at(3));
	//Eigen::Vector3d transGuess01 = TransToPose(transGuess0.inverse() * transGuess1);
	////std::cout << transGuess01.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess01, vCorrPoints01, transFinal01);
	//std::cout << transFinal01.transpose() << std::endl;

	//Eigen::Vector3d transGuess02 = TransToPose(transGuess0.inverse() * transGuess2);
	////std::cout << transGuess02.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess02, vCorrPoints02, transFinal02);
	//std::cout << transFinal02.transpose() << std::endl;

	//Eigen::Vector3d transGuess03 = TransToPose(transGuess0.inverse() * transGuess3);
	////std::cout << transGuess03.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess03, vCorrPoints03, transFinal03);
	//std::cout << transFinal03.transpose() << std::endl;

	//Eigen::Vector3d transGuess12 = TransToPose(transGuess1.inverse() * transGuess2);
	////std::cout << transGuess12.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess12, vCorrPoints12, transFinal12);
	//std::cout << transFinal12.transpose() << std::endl;

	//Eigen::Vector3d transGuess13 = TransToPose(transGuess1.inverse() * transGuess3);
	////std::cout << transGuess13.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess13, vCorrPoints13, transFinal13);
	//std::cout << transFinal13.transpose() << std::endl;

	//Eigen::Vector3d transGuess23 = TransToPose(transGuess2.inverse() * transGuess3);
	////std::cout << transGuess23.transpose() << std::endl;
	//G2O_OptimizerICP(transGuess23, vCorrPoints23, transFinal23);
	//std::cout << transFinal23.transpose() << std::endl;

	transFinal01 = GetRigidTransform(vCorrPoints01);
	transFinal02 = GetRigidTransform(vCorrPoints02);
	transFinal03 = GetRigidTransform(vCorrPoints03);
	transFinal12 = GetRigidTransform(vCorrPoints12);
	transFinal13 = GetRigidTransform(vCorrPoints13);
	transFinal23 = GetRigidTransform(vCorrPoints23);
	//std::cout << transFinal01.transpose() << std::endl << std::endl;
	//std::cout << transFinal02.transpose() << std::endl << std::endl;
	//std::cout << transFinal03.transpose() << std::endl << std::endl;
	//std::cout << transFinal12.transpose() << std::endl << std::endl;
	//std::cout << transFinal13.transpose() << std::endl << std::endl;
	//std::cout << transFinal23.transpose() << std::endl << std::endl;

	std::vector<Eigen::Vector3d> vv3dLiDARFinalPose2;
	std::vector<Fra2FraTransEdge> vFra2FraTransEdges;
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(0, 1, transFinal01,static_cast<double>(vCorrPoints01.size())));
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(0, 2, transFinal02, static_cast<double>(vCorrPoints02.size())));
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(0, 3, transFinal03, static_cast<double>(vCorrPoints03.size())));
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(1, 2, transFinal12, static_cast<double>(vCorrPoints12.size())));
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(1, 3, transFinal13, static_cast<double>(vCorrPoints13.size())));
	vFra2FraTransEdges.push_back(Fra2FraTransEdge(2, 3, transFinal23, static_cast<double>(vCorrPoints23.size())));

	G2O_OptimizerFra2Fra(vv3dLiDARInitialPoseGuess, vFra2FraTransEdges, vv3dLiDARFinalPose2);
	AnalyzeError(vv3dLiDARRealPose, vv3dLiDARInitialPoseGuess, vv3dLiDARFinalPose2);
	//////////////Frame to Frame Edge G2O//////////////////

	return 1;
}


