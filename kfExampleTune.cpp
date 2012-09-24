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
#include "generateData.h"

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

#define USE_GNUPLOT

const double INTERVALS = 20;
const double dt = 0.5;

double runSimulation(std::vector<StatePair> const& data, const double measurementVariance, const double processModelVariance) {

	/// We want a simple 2d state
	typedef SimpleState<2> state_t;

	/// Our process model is the simplest possible: doesn't change the mean
	typedef ConstantProcess<2, state_t> process_t;

	/// Create a kalman filter instance with our chosen state and process types
	KalmanFilter<state_t, process_t> kf;

	/// Set our process model's variance
	kf.processModel.sigma = state_t::VecState::Constant(processModelVariance);

	double sumSquaredError = 0;

	for (unsigned int i = 0; i < data.size(); ++i) {
		/// Predict step: Update Kalman filter by predicting ahead by dt
		kf.predict(dt);

		/// "take a measurement" - in this case, noisify the actual measurement
		AbsoluteMeasurement<state_t> meas;
		meas.measurement = data[i].second;
		meas.covariance = Eigen::Vector2d::Constant(measurementVariance).asDiagonal();

		/// Correct step: incorporate information from measurement into KF's state
		kf.correct(meas);

		Eigen::Vector2d pos(data[i].first);
		double squaredError = (pos[0] - kf.state.x[0]) * (pos[0] - kf.state.x[0]);
		sumSquaredError += squaredError;
	}
	return sumSquaredError;
}

void runWindow(std::vector<StatePair> const& data, double lowMVar, double highMVar, double lowPVar, double highPVar, int recursionsRemaining = 0) {
	std::stringstream ss;
#ifdef USE_GNUPLOT
	static bool doneOutput = false;
	std::ostream & output(doneOutput ? ss : std::cout);
	std::string separator(" ");
	doneOutput = true;
#else
	std::ostream & output( (recursionsRemaining == 0) ? std::cout : ss);
	std::string separator(",");
#endif


	double minErr = 10000;
	double bestMVar = lowMVar;
	double bestPVar = lowPVar;
#ifndef USE_GNUPLOT
	// Output column headers
	output << separator;
	for (double pVar = lowPVar; pVar < highPVar; pVar += (highPVar - lowPVar) / INTERVALS) {
		output << pVar << separator;
	}

	output << "process variance" << std::endl;
#endif

	const double dMVar = (highMVar - lowMVar) / INTERVALS;
	const double dPVar = (highPVar - lowPVar) / INTERVALS;

	for (double mVar = lowMVar; mVar < highMVar; mVar += dMVar) {
#ifndef USE_GNUPLOT
		/// row headers
		output << mVar;
#endif

		for (double pVar = lowPVar; pVar < highPVar; pVar += dPVar) {
			double err = runSimulation(data, mVar, pVar);
#ifdef USE_GNUPLOT
			//std::cout << mVar << " " << pVar << " " << err << std::endl;
#endif
			output << separator << err;
			if (err < minErr) {
				bestMVar = mVar;
				bestPVar = pVar;
				minErr = err;
			}
		}
		output << std::endl;
	}
	output << "measurement variance" << std::endl;

	std::cerr << std::endl;
	std::cerr << "Best found in the grid of parameters: " << std::endl;
	std::cerr << "Measurement Variance: " << bestMVar << std::endl;
	std::cerr << "Process variance: " << bestPVar << std::endl;
	std::cerr << "Sum squared error: " << minErr << std::endl;
	std::cerr << std::endl;
	
	if (recursionsRemaining > 0) {
		std::cerr << "Recursing..." << std::endl << std::endl;
		runWindow(data,
			bestMVar - dMVar,
			bestMVar + dMVar,
			bestPVar - dPVar,
			bestPVar + dPVar,
			recursionsRemaining -1);
	}

}

int main(int /*argc*/, char * /*argv*/[]) {

	std::vector<StatePair> data = generateLineData();

	double lowMVar = 0;
	double highMVar = 9.0;
	double lowPVar = 0;
	double highPVar = 11;

#ifdef USE_GNUPLOT
	//std::cout << "mVar pVar err" << std::endl;
#endif
	runWindow(data, lowMVar, highMVar, lowPVar, highPVar, 3);

	return 0;
}
