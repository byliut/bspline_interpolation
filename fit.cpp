#include "fit.hpp"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/distances.h>

using namespace std;
typedef pcl::PointXYZ PointXYZ;

namespace {
	constexpr double eps = 1e-7;
}
//this program consists of 3 part:
//1.parameterization
//2.Fitter::interpolation, via line interpolation, in u and v.

//
vector<pcl::PointXYZ> line_interpolation(const vector<PointXYZ>& data,	int m,
	const vector<double>& para_value, const Bspline& b)
{
	int n = data.size();
	int p = 3;
	Eigen::MatrixXf M(n, m + 1);
	Eigen::MatrixXf D(n, 3);
	for (int i = 0; i < n; i++) {
		auto nik = b.Nik(p + 1, para_value[i]);
		for (int j = 0; j < m + 1; j++)
			M(i, j) = nik[j];
		for (int j = 0; j < 3; j++)
			D(i, j) = data[i].data[j];
	}
	Eigen::MatrixXf P = M.lu().solve(D);//keast square, use LU decomposition to solve the control vertices
	//transform the solved P to pointxyz type
	vector<pcl::PointXYZ> res(m + 1);
	for (int i = 0; i < m + 1; i++) {
		res[i].x = P(i, 0);
		res[i].y = P(i, 1);
		res[i].z = P(i, 2);
	}
	return res;
}

//save the first interpolate result
void save1interPoints(const vector<vector<PointXYZ>>& controlPts, const std::string& filename) {
	// Create a PointCloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Iterate over the control points grid and add them to the cloud
	for (const auto& row : controlPts) {  //for row, traverse each row in current controlPts
		for (const auto& point : row) {  //for point, traverse each point in current row
			pcl::PointXYZ pcl_point;
			pcl_point.x = point.x;
			pcl_point.y = point.y;
			pcl_point.z = point.z;
			cloud->points.push_back(pcl_point);
		}
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;

	// Save the control points to a PLY file
	if (pcl::io::savePLYFileASCII(filename, *cloud) == -1) {
		std::cerr << "Error saving control points PLY file: " << filename << std::endl;
	}
	else {
		std::cout << "Control points saved to: " << filename << std::endl;
	}
}


/*data: original 3d data. m, n: (ctrl pts num)-1 in u, v
controlPts: output ctrl pts grid (m+1)*(n+1) us,vs:knots vector in u, v */
bool Fitter::interpolation(const vector<vector<pcl::PointXYZ>>& data, int m, int n,
	vector<vector<pcl::PointXYZ>>* controlPts,
	vector<double>* us, vector<double>* vs,	Bspline_Surface** knots_vec)
{
	int p = 3, q = 3;
	*knots_vec = get_parameter(data, m, n, us, vs);
	int num_col = vs->size();
	int num_row = us->size();
	vector<vector<pcl::PointXYZ>> Q(num_col, vector<PointXYZ>(m + 1));//Q存放中间控制点
	//line interpolation in group/row direction
	for (int col = 0; col < num_col; col++) {
		vector<pcl::PointXYZ> col_data(num_row);
		for (int row = 0; row < num_row; row++) {
			col_data[row] = data[row][col];
		}
		(*knots_vec)->u_knots_vec;//access a struct member
		Q[col] = line_interpolation(col_data, m, *us, (*knots_vec)->u_knots_vec);
	}
	save1interPoints(Q, "qcontrol.ply");
	
	//line interpolation in row direction
	for (int row = 0; row < m + 1; row++) {
		vector<pcl::PointXYZ> row_data(num_col);
		for (int col = 0; col < num_col; col++) {
			row_data[col] = Q[col][row];
		}
		(*controlPts)[row] = line_interpolation(row_data, n, *vs, (*knots_vec)->v_knots_vec);
	}	
	return true;	
}

 //1.Parameterization for u and v directions: uniform, chordlength, centripetal
 //2.Generation of knot vectors us, vs : uniform or average.
Bspline_Surface* Fitter::get_parameter(const vector<vector<PointXYZ>>& data,
	 int m, int n, vector<double>* us, vector<double>* vs)
	const{
	int p = 3, q = 3;
	int h = data.size();//group length, h=5
	int w = data[0].size();//curve length, w=10
	vector<vector<double>> uvs(h, vector<double>(w, 0));//knots vector of all points	
	vector<double> knots_u;//knots vector in u direction
	vector<double> knots_v;//knots vector in v direction
	auto f_uniform = [](const vector<PointXYZ>& points) {
		int n = points.size();
		vector<double> para_value(n);
		double delta = 1.0 / (n - 1);
		double sum = 0;
		for (int i = 0; i < n; i++) {
			para_value[i] = sum;
			sum += delta;
		}
		para_value[n - 1] = 1.0;
		return para_value;
	};
	auto f_chordlength = [](const vector<PointXYZ>& points) {
		int n = points.size();
		vector<double> arcs;
		double L = 0;
		for (int i = 1; i < n; i++) {
			double Li = pcl::euclideanDistance(points[i], points[i - 1]); // Use pcl::euclideanDistance instead of glm::length
			arcs.emplace_back(Li);
			L += Li;
		}
		vector<double> para_value(n, 0);
		double sum = 0;
		for (int i = 1; i < n; i++) {
			sum += arcs[i - 1];
			para_value[i] = sum / L;
		}
		para_value[n - 1] = 1.0;
		return para_value;
	};
	auto f_centripetal = [](const vector<PointXYZ>& points, double alpha) {
		int n = points.size();
		vector<double> arcs;
		double L = 0;
		for (int i = 1; i < n; i++) {
			double Li = std::pow(pcl::euclideanDistance(points[i], points[i - 1]), alpha); // Use pcl::euclideanDistance instead of glm::length
			arcs.emplace_back(Li);
			L += Li;
		}
		vector<double> para_value(n, 0);
		double sum = 0;
		for (int i = 1; i < n; i++) {
			sum += arcs[i - 1];
			para_value[i] = sum / L;
		}
		para_value[n - 1] = 1.0;
		return para_value;
	};
	//generation of knot vectors: uniform and average
	auto f_uniform_knots = [](const vector<double>& us, int p, int n) {
		int m = p + n + 1;
		vector<double> para_value(m + 1);
		for (int i = 0; i <= p; i++)
			para_value[i] = 0;
		for (int i = m - p; i <= m; i++)
			para_value[i] = 1.0;
		double delta = 1.0 / (n - p + 1);
		double sum = delta;
		for (int i = p + 1; i <= n; i++) {
			para_value[i] = sum;
			sum += delta;
		}
		return para_value;
	};
	auto f_average_knots = [](const vector<double>& us, int p, int n) {
		int m = p + n + 1;
		vector<double> para_value(m + 1);
		for (int i = 0; i <= p; i++)
			para_value[i] = 0;
		for (int i = m - p; i <= m; i++)
			para_value[i] = 1.0 + eps;			
		int step = us.size() - 1 - n + p;
		double inv_p = 1.0 / step;
		for (int j = 1; j <= n - p; j++) {
			double sum = 0.f;
			for (int i = j; i <= j + step - 1; i++) {
				sum += us[i];
			}
			para_value[j + p] = inv_p * sum;
		}
		return para_value;
	};

	if (parameter_method != universal) {
		vector<pcl::PointXYZ> points(h);
		for (int j = 0; j < w; j++) {//这层循环是求所有row的参数
			for (int i = 0; i < h; i++) {//这层循环是求row/组方向参数
				points[i] = data[i][j];
			}
			vector<double> para_value;
			if (parameter_method == uniformly_space) {
				para_value = f_uniform(points);
			}
			else if (parameter_method == chordlength) {
				para_value = f_chordlength(points);
			}
			else if (parameter_method == centripetal) {
				para_value = f_centripetal(points, alpha);
			}
			else {
				assert(0);
			}
			for (int i = 0; i < h; i++) {//把计算的参数值赋给uvs
				uvs[i][j] = para_value[i];
			}
		}
		//这个循环计算每个row的平均值，作为最终row方向的参数值
		for (int i = 0; i < h; i++) {
			double sum = 0;
			for (int j = 0; j < w; j++) {
				sum += uvs[i][j];
			}
			(*us)[i] = sum / w;
		}

		points.resize(w);//调整大小。计算完一个方向在计算另一个方向
		for (int i = 0; i < h; i++) {//这层循环求所有col/曲线 方向
			for (int j = 0; j < w; j++) {//这层循环求一个col/曲线 方向
				points[j] = data[i][j];
			}
			vector<double> para_value;
			if (parameter_method == uniformly_space) {
				para_value = f_uniform(points);
			}
			else if (parameter_method == chordlength) {
				para_value = f_chordlength(points);
			}
			else if (parameter_method == centripetal) {
				para_value = f_centripetal(points, alpha);
			}
			else {
				assert(0);
			}
			for (int j = 0; j < w; j++) {
				uvs[i][j] = para_value[j];
			}
		}

		//这个循环计算每个col的平均值，作为最终col方向的参数值
		for (int j = 0; j < w; j++) {
			double sum = 0;

			for (int i = 0; i < h; i++) {
				sum += uvs[i][j];
			}

			(*vs)[j] = sum / h;
		}

		if (knot_generation == uniform) {
			knots_u = f_uniform_knots(*us, p, m);
			knots_v = f_uniform_knots(*vs, q, n);
		}
		else if (knot_generation == average) {
			knots_u = f_average_knots(*us, p, m);
			knots_v = f_average_knots(*vs, q, n);
		}
		else {
			assert(0);
		}
	}
	else {
		/*
		universal method can not be used to approximate
		here we assume m == us.size()-1
		*/
		constexpr int sample_num = 8;
		// u
		{
			knots_u = f_uniform_knots(*us, p, m);
			Bspline u_knots_vec(m, p + 1, knots_u);
			(*us)[0] = 0;
			(*us)[m] = 1;
			for (int i = 1; i < m; i++) {
				double delta = (knots_u[i + p + 1] - knots_u[i]) / sample_num;
				double begin = knots_u[i];
				double max_value = -1;
				double max_u = begin;
				for (int s = 0; s < sample_num; s++) {
					double value = u_knots_vec.Nik(i, p + 1, begin);
					if (value > max_value) {
						max_value = value;
						max_u = begin;
					}
					begin += delta;
				}
				(*us)[i] = max_u;
			}
		}
		// v
		{
			knots_v = f_uniform_knots(*vs, q, n);
			Bspline v_knots_vec(n, q + 1, knots_v);
			(*vs)[0] = 0;
			(*vs)[n] = 1;
			for (int i = 1; i < n; i++) {
				double delta = (knots_v[i + q + 1] - knots_v[i]) / sample_num;
				double begin = knots_v[i];
				double max_value = -1;
				double max_v = begin;
				for (int s = 0; s < sample_num; s++) {
					double value = v_knots_vec.Nik(i, q + 1, begin);
					if (value > max_value) {
						max_value = value;
						max_v = begin;
					}
					begin += delta;
				}
				(*vs)[i] = max_v;
			}
		}
	}
	// p q is degree but need order
	Bspline_Surface* knots_vec = new Bspline_Surface(m, n, p + 1, q + 1, knots_u, knots_v);
	return knots_vec;
}
