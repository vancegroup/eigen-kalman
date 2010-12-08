/** @file KalmanFilter.h
	@brief

	@date 2010

	@author
	Ryan Pavlik
	<rpavlik@iastate.edu> and <abiryan@ryand.net>
	http://academic.cleardefinition.com/
	Iowa State University Virtual Reality Applications Center
	Human-Computer Interaction Graduate Program
*/


#pragma once
#ifndef INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB
#define INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

#include <Eigen/Eigen>

template<unsigned int dimension>
class SimpleState {
public:
	static const int DIM = dimension;
	typedef Eigen::Matrix<double, dimension, 1> Vector;
	typedef Eigen::Matrix<double, dimension, dimension> Matrix;
	
	Vector vec;
	Matrix covariance;
	
	inline SimpleState() :
		vec(Vector::Zero()),
		covariance(Matrix::Identity()) {
		
	}
	
};

template<int dimension, class StateType>
class ConstantProcess {
public:
	typedef Eigen::Matrix<double, StateType::DIM, 1> StateVector;
	typedef Eigen::Matrix<double, StateType::DIM, StateType::DIM> StateMatrix;
	
	
	/// Noise per second, per component
	StateVector sigma;
	StateMatrix noise;
	StateMatrix jacobian;
	
	inline ConstantProcess() :
		sigma(StateVector::Zero()),
		noise(StateMatrix::Zero()),
		jacobian(StateMatrix::Identity()) 
	{}
	
	StateMatrix const& getJacobian(StateType const& /*state*/, const double /*dt*/) {
		/// No change over time due to process model
		return jacobian;
	}
	
	void updateState(StateType const& /*state*/, const double /*dt*/) {
		/// no-op - no change due to process model
	}
	
	StateMatrix const& getNoiseCovariance(const double dt) {
		for (int i = 0; i < StateType::DIM; ++i) {
			noise(i, i) = dt * sigma(i);
		}
		return noise;
	}
		
	void updateFromMeasurement(StateType & state, StateVector const & innovation) {
		state.vec += innovation;
	}
	
};


template<class StateType, class ProcessModelType>
class KalmanFilter {
	public:
	typedef Eigen::Matrix<double, StateType::DIM, StateType::DIM> StateCovarianceType;
	
	
	
	StateType state;
	ProcessModelType processModel;
	
	void predict(double dt) {
		const StateCovarianceType A(processModel.getJacobian(state, dt));
		state.covariance = A * state.covariance * A.transpose() + processModel.getNoiseCovariance(dt);
		/// @todo symmetrize?
		processModel.updateState(state, dt);		
	}
	
	template<class MeasurementType>
	void correct(MeasurementType & m) {
		/// @todo implement
	}
	
	
};

#endif // INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

