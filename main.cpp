#include "fit.hpp"
#include "header_mybspline1.h"

//v1.1 remove all the opengl visualize module
//ver. 20241129, log: change vec3 into pcl::PointXYZ; add pcl::visualize

int main(int argc, char** argv)
{
    int p = 3, q = 3;//degree in u v direction
	//command line: bspline data.txt, the first row in data.txt indicates the row and col, ie. 5 6
    int num_row, num_col;//every col data form a group
	vector<vector<pcl::PointXYZ>> *data;
	data = new vector<vector<pcl::PointXYZ>>();
	//interpolation does not need the paramater m and n
	//the first is the group num, the second is the num in each group
	if (!read_file(argv[1], data, &num_row, &num_col))
	{
		std::cout << "File not exist!" << std::endl;
		return -1;
	}

	//control points grid, row ¡Á col
	controlPts = new vector<vector<pcl::PointXYZ>>();
	int m = num_row - 1;
	int n = num_col - 1;
	controlPts->resize(m + 1);
	for (int i = 0; i < m + 1; i++) 
		(*controlPts)[i].resize(n + 1);

	dus = new vector<double>(num_row);
	dvs = new vector<double>(num_col);
 
	fitter.interpolation(*data, m, n, controlPts, dus, dvs, &knots_vec);
	saveControlPoints(*controlPts, "control_points.ply");
	std::string filename = "b_spline_surface.ply";
	//int u_samples = 80;  // Number of samples in the u-direction
	//int v_samples = 100;  // Number of samples in the v-direction	
	//generatePointCloud(*bs, *controlPts, u_samples, v_samples, filename);
	PointCloud::Ptr interpolated_cloud(new PointCloud);
	interpolated_cloud = saveInterpolatedSurfacePoints(*knots_vec, "interpolated_surface_points.ply", *controlPts, 0.02f);
	saveInterpolatedData(*controlPts, "interpolated_data.txt");

	visualize(interpolated_cloud);
	
	//delete controlPts;
	return 0;
}

