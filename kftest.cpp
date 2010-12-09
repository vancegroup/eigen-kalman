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
// - none

int main(int argc, char * argv[]) {
	typedef SimpleState<2> state_t;
	typedef ConstantProcess<2, state_t> process_t;
	KalmanFilter<state_t, process_t> kf;
	double dt = 0.5;
	for (double t = 0; t < 5.0; t+= dt) {
		kf.predict(dt);
		double noise = (std::rand() % 5) / 5.0;
		
		Eigen::Vector2d pos(Eigen::Vector2d::Constant(t));
		AbsoluteMeasurement<state_t> meas;
		meas.x = (pos + Eigen::Vector2d::Constant(noise)).eval();
		kf.correct(meas);
		std::cout << kf.state.vec << std::endl;
	}
	return 0;

}