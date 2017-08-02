#ifndef UTILS_HPP
#define UTILS_HPP

#include <math.h>
#include <iostream>

using namespace std;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline double evaluate_coefficients(vector<double> coeff, double t) {
	double value = 0;
	for(int i=0; i<coeff.size(); i++) {
		value += coeff[i]*pow(t, i);
	}
	return value;
}

inline vector<double> differentiate(vector<double> coefficients) {
    vector<double> new_cos = {};
		for(int i=1; i<coefficients.size(); i++) {
			new_cos.push_back(i * coefficients[i]);
		}
    return new_cos;
}

inline double logistic(double x) {
	return 2.0 / (1 + exp(-x)) - 1.0;
}

#endif
