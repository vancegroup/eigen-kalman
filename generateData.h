/**
	@file generateData.h
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
#ifndef INCLUDED_generateData_GUID_07DCD19D_CE5B_406B_BDCB_6BEAC4F07443
#define INCLUDED_generateData_GUID_07DCD19D_CE5B_406B_BDCB_6BEAC4F07443

// Internal Includes
// - none

// Library/third-party includes
#include <Eigen/Core>

// Standard includes
#include <utility>
#include <vector>
#include <cmath>

typedef std::pair<Eigen::Vector2d, Eigen::Vector2d> StatePair;

double noise(double noiseAmplitude = 3.0) {
	return ((std::rand() % 100) / 50.0 - 1.0) * noiseAmplitude;
}

std::vector<StatePair> generateLineData(const double dt = 0.5, const double maxTime = 50.0, double noiseAmplitude = 3.0) {
	std::vector<StatePair> ret;
	for (double t = 0; t < maxTime; t += dt) {
		Eigen::Vector2d err;
		err[0] = noise(noiseAmplitude);
		err[1] = noise(noiseAmplitude);
		Eigen::Vector2d pos(Eigen::Vector2d::Constant(t));
		Eigen::Vector2d measurement(pos + err);
		ret.push_back(StatePair(pos, measurement));
	}
	return ret;
}

std::vector<StatePair> generateSineData(const double dt = 0.5, const double maxTime = 50.0, double noiseAmplitude = 3.0, double maxHeight = 10.0) {
	std::vector<StatePair> ret;
	const double timeScale = 3.141592653589 / maxTime;
	for (double t = 0; t < maxTime; t += dt) {
		Eigen::Vector2d err;
		err[0] = noise(noiseAmplitude);
		err[1] = noise(noiseAmplitude);

		Eigen::Vector2d pos;
		pos[0] = t;
		pos[1] = std::sin(t * timeScale) * maxHeight;

		Eigen::Vector2d measurement(pos + err);
		ret.push_back(StatePair(pos, measurement));
	}
	return ret;
}


#endif // INCLUDED_generateData_GUID_07DCD19D_CE5B_406B_BDCB_6BEAC4F07443

