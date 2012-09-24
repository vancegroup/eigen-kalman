/** @file KalmanFilter.h
	@brief KalmanFilter classes using the Eigen math template library,
	inspired by equivalent classes in TAG (using TooN for math)



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

#include <Eigen/Core>
#include <Eigen/LU>

#if EIGEN_VERSION_AT_LEAST(2, 9, 0)
// Eigen 3.0 series
#	define eikfLUType FullPivLU
#	define eikfLUFunc fullPivLu

#else
// Eigen 2.0.x
#	define eikfLUType LU
#	define eikfLUFunc lu
#endif

#include <stdexcept>
namespace eigenkf {

	template<int dimension>
	class SimpleState {
		public:
			static const int DIM = dimension;
			typedef Eigen::Matrix<double, dimension, 1> VecState;
			typedef Eigen::Matrix<double, dimension, dimension> MatStateState;

			VecState x;
			MatStateState covariance;

			SimpleState() :
				x(VecState::Zero()),
				covariance(MatStateState::Identity()) { }

	};

	template<int dimension, class StateType>
	class ConstantProcess {
		public:
			typedef Eigen::Matrix<double, StateType::DIM, 1> VecState;
			typedef Eigen::Matrix<double, StateType::DIM, StateType::DIM> MatStateState;


			/// Noise per second, per component
			VecState sigma;
			MatStateState noise;
			MatStateState jacobian;

			inline ConstantProcess() :
				sigma(VecState::Zero()),
				noise(MatStateState::Zero()),
				jacobian(MatStateState::Identity())
			{}

			MatStateState const& getJacobian(StateType const& /*state*/, const double /*dt*/) {
				/// No change over time due to process model
				return jacobian;
			}

			void updateState(StateType const& /*state*/, const double /*dt*/) {
				/// no-op - no change due to process model
			}

			MatStateState const& getNoiseCovariance(const double dt) {
				noise = (dt * sigma).asDiagonal();
				return noise;
			}

			void updateFromMeasurement(StateType & state, VecState const & innovation) {
				state.x += innovation;
			}

	};

	template<class StateType>
	class AbsoluteMeasurement {
		public:
			static const int DIM = StateType::DIM;
			typedef Eigen::Matrix<double, DIM, 1> VecMeas;
			typedef Eigen::Matrix<double, DIM, 1> VecState;
			typedef Eigen::Matrix<double, DIM, DIM> MatMeasMeas;
			typedef Eigen::Matrix<double, DIM, DIM> MatMeasState;

			VecMeas measurement;

			MatMeasState jacobian;
			MatMeasMeas covariance;

			AbsoluteMeasurement() :
				measurement(VecMeas::Zero()),
				jacobian(MatMeasState::Identity()),
				covariance(MatMeasMeas::Identity()) {}


			MatMeasState const& getJacobian(StateType const& /*state*/) {
				return jacobian;
			}

			/// Measurement noise covariance, aka uncertainty
			/// in measurement
			MatMeasMeas const& getCovariance(StateType const& /*state*/) {
				return covariance;
			}

			VecState const getInnovation(StateType const& state) {
				return measurement - state.x;
			}

	};

	template<class StateType, class ProcessModelType>
	class KalmanFilter {
		public:
			typedef Eigen::Matrix<double, StateType::DIM, StateType::DIM> MatStateState;
			typedef Eigen::Matrix<double, StateType::DIM, 1> VecState;



			StateType state;
			ProcessModelType processModel;

			void predict(double dt) {
				const MatStateState A(processModel.getJacobian(state, dt));
				state.covariance = A * state.covariance * A.transpose() + processModel.getNoiseCovariance(dt);
				/// @todo symmetrize?
				processModel.updateState(state, dt);
			}

			template<class MeasurementType>
			void correct(MeasurementType & m) {
				typedef Eigen::Matrix<double, MeasurementType::DIM, StateType::DIM> MatMeasState;
				typedef Eigen::Matrix<double, MeasurementType::DIM, MeasurementType::DIM> MatMeasMeas;
				typedef Eigen::Matrix<double, MeasurementType::DIM, 1> VecMeas;
				/// @todo implement
				const MatMeasState & H = m.getJacobian(state);
				const MatMeasMeas & R = m.getCovariance(state);
				const VecState innovation = m.getInnovation(state);
				const MatMeasMeas S = H * state.covariance * H.transpose() + R;
				/*
					Eigen::LDLT<MatMeasMeas> ldltOfS = S.ldlt();

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
				Eigen::eikfLUType<MatMeasMeas> luOfS = S.eikfLUFunc();
				MatStateState K = state.covariance * H.transpose() * luOfS.inverse();
				processModel.updateFromMeasurement(state, K * innovation);
				state.covariance = (MatStateState::Identity() - K * H) * state.covariance;

			}


	};

} // end of namespace eigenkf

#endif // INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

