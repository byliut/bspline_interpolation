#pragma once
#include <vector>
#include <assert.h>
#include <pcl/point_types.h>

using namespace std;
typedef pcl::PointXYZ PointXYZ;

struct Bspline {	
	//构造函数，n = 控制点个数 - 1， k = order = degree - 1
	Bspline(int n_, int k_, const vector<double>& knots_) :
		n(n_), k(k_), knots(knots_) {
		assert(n + k + 1 <= knots.size());//check the validity of the B-spline
	}
	//传入控制点和参数t
	pcl::PointXYZ  operator()(const vector<PointXYZ>& controlPts, double t) const {
		if (t < knots[k - 1] || t >= knots[n + 1])
			return pcl::PointXYZ(0, 0, 0);		
		auto&& nik = Nik(k, t); //calls Nik(k,t) to calc the basis function at t
		pcl::PointXYZ pt(0, 0, 0);
		for (int i = 0; i < nik.size(); i++) {  
			pt.x += nik[i] * controlPts[i].x;
			pt.y += nik[i] * controlPts[i].y;
			pt.z += nik[i] * controlPts[i].z;
		}
		return pt;
	}
	//计算单个基函数, k = 3, i = 0,1,2...
	double Nik(int i, int k, double t) const
	{
		if (t < knots[i] || t > knots[i + k]) {
		//if (t < knots[i] || t >= knots[i + k]) { //if t is not in the knot interval, return 0
			return 0;
		}
		vector<double> dp(n + 1, 0);//dp to storage the recursion value of the base function
		int j = 0;
		for (j = 0; j <= n; j++) {
			if (t >= knots[j] && t < knots[j + 1]) {
				dp[j] = 1;
				break;
			}
		}
		for (int kk = 2; kk <= k; kk++) {
			for (int ii = 0; ii <= n; ii++) {
				double a = dp[ii];
				double b = ii == n ? 0 : dp[ii + 1];
				double denominator1 = knots[ii + k - 1] - knots[ii];
				double factor1 = t - knots[ii];
				if (denominator1 == 0) 
					factor1 = 0;
				else 
					factor1 /= denominator1;
				double denominator2 = knots[ii + k] - knots[ii + 1];
				double factor2 = knots[ii + k] - t;
				if (denominator2 == 0) 
					factor2 = 0;
				else 
					factor2 /= denominator2;
				dp[ii] = factor1 * a + factor2 * b;
			}
		}
		return dp[i];
	}
	//计算所有的基函数
	vector<double> Nik(int k, double t) const
	{
		vector<double> dp(n + 1, 0);
		if (t < knots[k - 1] || t >= knots[n + 1]) {
			return dp;
		}
		int j = 0;
		for (j = 0; j <= n; j++) {
			if (t >= knots[j] && t < knots[j + 1]) {
				dp[j] = 1;
				break;
			}
		}
		for (int kk = 2; kk <= k; kk++) {
			for (int ii = 0; ii <= n; ii++) {
				double a = dp[ii];
				double b = ii == n ? 0 : dp[ii + 1];
				//三元运算符ternary operator,condition ? value_if_true : value_if_false;
				//if condition is true, return value_if_true, else ,return value_if_false
				//if ii == n, b = 0, else b = 1
				double denominator1 = knots[ii + k - 1] - knots[ii];
				double factor1 = t - knots[ii];
				if (denominator1 == 0) 
					factor1 = 0;
				else 
					factor1 /= denominator1;
				double denominator2 = knots[ii + k] - knots[ii + 1];
				double factor2 = knots[ii + k] - t;
				if (denominator2 == 0) {
					factor2 = 0;
				}
				else {
					factor2 /= denominator2;
				}
				dp[ii] = factor1 * a + factor2 * b;
			}
		}
		return dp;
	}

	int n, k;
	const vector<double> knots;
};

struct Bspline_Surface {
	Bspline_Surface(int m_, int n_, int p_, int q_,
		const vector<double>& knots_u_, 
		const vector<double>& knots_v_)
		:u_knots_vec(m_, p_, knots_u_),	v_knots_vec(n_, q_, knots_v_) {	}	
	//input: ctrl pts, u, v return: bspline surface value in (u,v)
	pcl::PointXYZ operator()(const vector<vector<PointXYZ>>& controlPts,
		double u, double v) const {
		vector<PointXYZ> tmp;
		for (auto&& p : controlPts) {
			PointXYZ vbspts = v_knots_vec(p, v);
			tmp.emplace_back(vbspts);
		}		
		pcl::PointXYZ ubspts = u_knots_vec(tmp, u);
		return ubspts;		
	}	
	const Bspline u_knots_vec, v_knots_vec;
};