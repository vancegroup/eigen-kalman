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
#include <Eigen/Cholesky>

#include <stdexcept>

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
		noise = (dt * sigma).asDiagonal();
		return noise;
	}
		
	void updateFromMeasurement(StateType & state, StateVector const & innovation) {
		state.vec += innovation;
	}
	
};

template<class StateType>
class AbsoluteMeasurement {
public:
	static const int DIM = StateType::DIM;
	typedef Eigen::Matrix<double, DIM, 1> Vector;
	typedef Eigen::Matrix<double, DIM, DIM> Matrix;
	Vector x;
	
	Matrix jacobian;
	Matrix covariance;
	
	AbsoluteMeasurement() :
		x(Vector::Zero()),
		jacobian(Matrix::Identity()),
		covariance(Matrix::Identity()) {}
	
	
	Matrix const& getJacobian(StateType const& state) {
		return jacobian;
	}
	
	/// Measurement noise covariance, aka uncertainty
	/// in measurement
	Matrix const& getCovariance(StateType const& state) {
		return covariance;
	}
	
	Vector const getInnovation(StateType const& state) {
		return state.vec - x;
	}
	
};

template<class StateType, class ProcessModelType>
class KalmanFilter {
	public:
	typedef Eigen::Matrix<double, StateType::DIM, StateType::DIM> Matrix;
	
	
	
	StateType state;
	ProcessModelType processModel;
	
	void predict(double dt) {
		const Matrix A(processModel.getJacobian(state, dt));
		state.covariance = A * state.covariance * A.transpose() + processModel.getNoiseCovariance(dt);
		/// @todo symmetrize?
		processModel.updateState(state, dt);		
	}
	
	template<class MeasurementType>
	void correct(MeasurementType & m) {
		/// @todo implement
		const Eigen::Matrix<double, MeasurementType::DIM, StateType::DIM> & H = m.getJacobian(state);
		const Eigen::Matrix<double, MeasurementType::DIM, MeasurementType::DIM> & R = m.getCovariance(state);
		const Eigen::Matrix<double, StateType::DIM, 1> innovation = m.getInnovation(state);
		const Eigen::Matrix<double, MeasurementType::DIM, MeasurementType::DIM> S = H * state.covariance * H.transpose() + R;
		
		Eigen::LDLT<Eigen::Matrix<double, MeasurementType::DIM, MeasurementType::DIM> > ldltOfS = S.ldlt();
		/*
		bool result = ldltOfS.solve(state.covariance * H.transpose(), &K);
		if (!result) {
			throw new std::runtime_error("cholesky failed!");
		}
		Matrix covCorrection;
		if (!ldltOfS.solve(state.covariance * H.transpose(), & covCorrection)) {
			throw new std::runtime_error("First ldlt failed!");
		}
		state.covariance -= covCorrection;
		
		Matrix stateInnovPart;
		if (!ldltOfS.solve(innovation, &stateInnovPart)) {
			throw new std::runtime_error("Second ldlt failed!");
		}
		 processModel.updateFromMeasurement(state, state.covariance * H.transpose() * stateInnovPart);

		 */
		Eigen::LU<Matrix> luOfS = S.lu();
		Matrix K = state.covariance * H.transpose() * luOfS.inverse();
		processModel.updateFromMeasurement(state, K * innovation);
		state.covariance = (Matrix::Identity() - K*H)*state.covariance;
		
	}
	
	
};

#endif // INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

