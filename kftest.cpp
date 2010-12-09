/**
	@file kftest.cpp
	@brief

	@date 2010

	@author
	Ryan Pavlik
	<rpavlik@iastate.edu> and <abiryan@ryand.net>
	http://academic.cleardefinition.com/
	Iowa State University Virtual Reality Applications Center
	Human-Computer Interaction Graduate Program
*/

// Internal Includes
// - none

// Library/third-party includes
#include <eigenkf/KalmanFilter.h>

// Standard includes
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>

using namespace eigenkf;

#define COL 10
#define NOISE_AMPLITUDE 3.0

double noise() {
	return ((std::rand() % 100) / 50.0 - 1.0) * NOISE_AMPLITUDE;
}

int main(int argc, char * argv[]) {
	//std::srand(std::time(NULL));

	/// We want a simple 2d state
	typedef SimpleState<2> state_t;

	/// Our process model is the simplest possible: doesn't change the mean
	typedef ConstantProcess<2, state_t> process_t;

	/// Create a kalman filter instance with our chosen state and process types
	KalmanFilter<state_t, process_t> kf;

	/// Set our process model's variance
	kf.processModel.sigma = state_t::VecState::Constant(0.5);

	double dt = 0.5;
	double sumSquaredError = 0;
	
	const double measurementVariance = NOISE_AMPLITUDE / 2.0;

	/// CSV header row
	std::cout << "actual,measurement,filtered,squared error" << std::endl;
	for (double t = 0; t < 50.0; t+= dt) {
		/// Predict step: Update Kalman filter by predicting ahead by dt
		kf.predict(dt);

		/// "take a measurement" - in this case, noisify the actual measurement
		Eigen::Vector2d pos(Eigen::Vector2d::Constant(t));
		AbsoluteMeasurement<state_t> meas;
		meas.measurement = (pos + Eigen::Vector2d(noise(), noise())).eval();
		meas.covariance = Eigen::Vector2d::Constant(measurementVariance).asDiagonal();

		/// Correct step: incorporate information from measurement into KF's state
		kf.correct(meas);

		/// Output info for csv
		double squaredError = (pos[0] - kf.state.x[0]) * (pos[0] - kf.state.x[0]);
		sumSquaredError += squaredError;
		std::cout << pos[0] << ",";
		std::cout << meas.measurement[0] << ",";
		std::cout << kf.state.x[0] << ",";
		std::cout << squaredError;
		std::cout << std::endl;

	}
	std::cerr << "Sum squared error: " << sumSquaredError << std::endl;
	return 0;

}
