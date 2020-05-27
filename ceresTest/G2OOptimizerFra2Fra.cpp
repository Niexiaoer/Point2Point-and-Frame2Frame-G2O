#include "G2OOptimizerFra2Fra.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>

void G2O_OptimizerFra2Fra(const std::vector<Eigen::Vector3d>& vv3dLiDARInitialPoseGuess, 
	const std::vector<Fra2FraTransEdge>& vFra2FraTransEdges, 
	std::vector<Eigen::Vector3d>& vv3dLiDARFinalPose)
{
	g2o::BlockSolverX::LinearSolverType* linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>());
	// create the block solver on the top of the linear solver
	g2o::BlockSolverX* blockSolver(new g2o::BlockSolverX(linearSolver));

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
	for (int i = 0;i < vFra2FraTransEdges.size();i++)
	{
		Fra2FraTransEdge edgeTmp = vFra2FraTransEdges.at(i);
		Pose3dEdge* edge = new Pose3dEdge();
		edge->setId(i);
		edge->setVertex(0, vPose3dVertexs.at(edgeTmp.ntargetFrameOffset));
		edge->setVertex(1, vPose3dVertexs.at(edgeTmp.nsourceFrameOffset));
		edge->setMeasurement(edgeTmp.transPose);
		//Eigen::Matrix3d InfoMatrix = Eigen::Matrix3d::Identity();
		//InfoMatrix *= 10.0;
		edge->setInformation(edgeTmp.InfoMatrix);
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
