#include "G2OOptimizerICP.h"

void G2O_OptimizerICP(const Eigen::Vector3d& v3dInitialTransGuess,
	const std::vector<CorrPoint>& vCorrPoints,
	Eigen::Vector3d& v3dFinalTrans)
{
	g2o::BlockSolverX::LinearSolverType* linearSolver(new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>());
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
	Trans3dVertex* v = new Trans3dVertex();
	v->setEstimate(v3dInitialTransGuess);
	v->setId(0);
	optimizer.addVertex(v);
	
	//Add Edge
	for (int i = 0;i < vCorrPoints.size();i++)
	{
		CorrPoint edgeTmp = vCorrPoints.at(i);
		CorrPointEdge* edge = new CorrPointEdge(edgeTmp.sourcePoint);
		edge->setId(i);
		edge->setVertex(0, v);
		edge->setMeasurement(edgeTmp.targetPoint);
		Eigen::Matrix2d InfoMatrix = Eigen::Matrix2d::Identity();
		InfoMatrix *= 10.0;
		edge->setInformation(InfoMatrix);
		optimizer.addEdge(edge);
	}
	optimizer.initializeOptimization();
	optimizer.optimize(1000);

	v3dFinalTrans = v->estimate();

}