#pragma once
#include "G2OOptimizer.h"
#include <g2o/core/base_unary_edge.h>

struct CorrPoint
{
	CorrPoint(Eigen::Vector2d _targetPoint,Eigen::Vector2d _sourcePoint)
	{
		targetPoint = _targetPoint;
		sourcePoint = _sourcePoint;
	}
	Eigen::Vector2d targetPoint;
	Eigen::Vector2d sourcePoint;
};

//Define TransMatrix Vertex
class Trans3dVertex :public g2o::BaseVertex<3, Eigen::Vector3d>
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

//Define Correspondence Point Edge
class CorrPointEdge :public g2o::BaseUnaryEdge<2, Eigen::Vector2d, Trans3dVertex>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CorrPointEdge(Eigen::Vector2d _v2dSourcePoint) :BaseUnaryEdge(),
	m_v2dSourcePoint(_v2dSourcePoint)
	{
	}

	void computeError()
	{
		const Trans3dVertex* v0 = static_cast<const Trans3dVertex*> (_vertices[0]);
		Eigen::Matrix3d TransMatrix = PoseToTransNIE(Eigen::Vector3d(v0->estimate()));

		Eigen::Vector3d v3dSourcePoint(m_v2dSourcePoint(0), m_v2dSourcePoint(1), 1.0);
		_error = _measurement - (TransMatrix * v3dSourcePoint).head<2>();
	}

	virtual bool read(std::istream& in) { return true; };
	virtual bool write(std::ostream& out)const { return true; };
private:
	Eigen::Vector2d m_v2dSourcePoint;
};

//G2O optimizerTraditional set
void G2O_OptimizerICP(const Eigen::Vector3d& v3dInitialTransGuess,
	const std::vector<CorrPoint>& vCorrPoints,
	Eigen::Vector3d& v3dFinalTrans);