/**
	@file kfExampleTune.cpp
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
#include <Eigen/Eigen>

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

double runSimulation(const double measurementVariance, const double processModelVariance) {
	/// Reset the random seed
	std::srand(200);

	/// We want a simple 2d state
	typedef SimpleState<2> state_t;

	/// Our process model is the simplest possible: doesn't change the mean
	typedef ConstantProcess<2, state_t> process_t;

	/// Create a kalman filter instance with our chosen state and process types
	KalmanFilter<state_t, process_t> kf;

	/// Set our process model's variance
	kf.processModel.sigma = state_t::VecState::Constant(processModelVariance);

	double dt = 0.5;
	double sumSquaredError = 0;

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

		double squaredError = (pos[0] - kf.state.x[0]) * (pos[0] - kf.state.x[0]);
		sumSquaredError += squaredError;
	}
	std::cout << std::setw(COL) << measurementVariance;
	std::cout << std::setw(COL) << processModelVariance;
	std::cout << std::setw(COL) << sumSquaredError;
	std::cout << std::endl;
	return sumSquaredError;
}

Eigen::Vector2d g_mVariance;
Eigen::Vector2d g_pVariance;
double g_splitMeasurement;
double g_splitProcess;

Eigen::Matrix2d g_errors;
int g_whichMVkeep = -1;
int g_whichPVkeep = -1;
double g_minErr = 10000000;

void checkCorner(int whichMV, int whichPV) {
	double err = runSimulation(g_mVariance[whichMV], g_pVariance[whichPV]);
	g_errors(whichMV, whichPV) = err;
	if (err < g_minErr) {
		g_whichMVkeep = whichMV;
		g_whichPVkeep = whichPV;
		g_minErr = err;
	}
}
int main(int argc, char * argv[]) {
	/// Starting windows to consider
	g_mVariance << 0, NOISE_AMPLITUDE * 2.0;
	g_pVariance << 0, 15.0;

	for (unsigned int iterations = 0; iterations < 10; ++iterations) {
		double err;

		checkCorner(0, 0);
		checkCorner(0, 1);
		checkCorner(1, 0);
		checkCorner(1, 1);

		g_splitMeasurement = g_mVariance.sum() / 2.0;
		g_splitProcess = g_pVariance.sum() / 2.0;

		std::cout << std::endl;
		std::cout << "After this iteration, our g_errors look like: " << std::endl;
		std::cout << g_errors << std::endl;
		std::cout << "We are keeping these parameters: " << std::endl;
		std::cout << "Measurement Variance: " << g_mVariance[g_whichMVkeep] << std::endl;
		std::cout << "Process variance: " << g_pVariance[g_whichPVkeep] << std::endl;
		std::cout << "Sum squared error: " << g_minErr << std::endl;
		std::cout << std::endl;

		/// Adjust window
		for (int whichMV = 0; whichMV < 2; ++whichMV) {
			for (int whichPV = 0; whichPV < 2; ++whichPV) {
				if (whichMV != g_whichMVkeep) {
					// update this measurement variance
					g_mVariance[whichMV] = splitMeasurement;
				}
				if (whichPV != g_whichPVkeep) {
					// update this process variance
					g_pVariance[whichPV] = splitProcess;
				}
			}
		}

	}

	return 0;

}
