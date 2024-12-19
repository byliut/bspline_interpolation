#pragma once
#include "bspline.hpp"

struct Fitter {
	/* This method computes a B-spline surface that exactly fits the input data (interpolation).
	The result is stored in a B-spline surface, *knots_vec, and the surface control points are
	stored in controlPts.*/
	bool interpolation(
		const vector<vector<pcl::PointXYZ>>& data,
		int m, 
		int n,
		vector<vector<pcl::PointXYZ>>* controlPts,
		vector<double>* us,
		vector<double>* vs,	
		Bspline_Surface** knots_vec
	);

	enum Parameter_Method{
		uniformly_space,
		chordlength,
		centripetal,
		universal,
	};
	enum Knot_Generation{
		uniform,
		average,
	};

	Parameter_Method parameter_method = chordlength;	
	Knot_Generation knot_generation = average;
	//if you don't assign a value, it will be the first, that is, uniformaly_space and uniform
	double alpha = 0.5f;
	
private:
	Bspline_Surface* get_parameter(
		const vector<vector<pcl::PointXYZ>>& data,
		int m,
		int n,		
		vector<double>* us,
		vector<double>* vs
	)const;
};