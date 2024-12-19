#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

namespace {
	Fitter fitter;
	Bspline_Surface* knots_vec = nullptr;

	vector<vector<PointXYZ>> * data, * controlPts;
	vector<double>* dus, * dvs;	
}

typedef pcl::PointCloud<PointXYZ> PointCloud;
//read 3D point clouds from local file, fill to "data", set the data's row and column 
bool read_file(const char* file_name, vector<vector<pcl::PointXYZ>>* data, int* row, int* col) {

	std::ifstream in(file_name);
	if (!in.is_open()) //check if the file is opened successfully
		return false;
	in >> (*row) >> (*col);//read the first two characters, assign to row and col
	data->resize(*row); //after resize, data has row vector<vec3>
	for (int i = 0; i < *row; i++) {
		(*data)[i].resize(*col);
	}
	//store the data in txt into data[i][j]
	for (int i = 0; i < *row; i++) {
		for (int j = 0; j < *col; j++) {
			pcl::PointXYZ p;
			in >> p.x >> p.y >> p.z;
			(*data)[i][j] = p;
		}
	}
	return true;
}


//save control points into ply
void saveControlPoints(const vector<vector<pcl::PointXYZ>>& controlPts, const std::string& filename) {
	// Create a PointCloud object
	PointCloud::Ptr cloud(new PointCloud);

	// Iterate over the control points grid and add them to the cloud
	for (const auto& row : controlPts) {
		for (const auto& point : row) {
			cloud->points.push_back(point);
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

PointCloud::Ptr saveInterpolatedSurfacePoints(const Bspline_Surface& knots_vec, const std::string& filename,
	const vector<vector<pcl::PointXYZ>>& controlPoints, double step) {
	PointCloud::Ptr cloud(new PointCloud);
	int i = 0;
	for (double u = 0; u <= 1.0f; u += step) {
		for (double v = 0; v <= 1.0f; v += step) {
			pcl::PointXYZ point = knots_vec(controlPoints, u, v); // Evaluate B-spline surface at (u, v)
			cloud->push_back(point);
			i++;
		}
		cloud->width = cloud->size();
		cloud->height = 1; // Point cloud is unorganized
		cloud->is_dense = true;
		//pcl::io::savePLYFile("interpolated_cloud_row.ply", *cloud);
		//std::cout << "Interpolated points saved to: " << filename << std::endl;
	}

	cloud->width = cloud->size();
	cloud->height = 1; // Point cloud is unorganized
	cloud->is_dense = true;

	pcl::io::savePLYFile(filename, *cloud);
	return cloud;
}

//PointCloud::Ptr saveInterpolatedPoints(const Bspline_Surface& bs, const std::string& filename,
//	const vector<vector<pcl::PointXYZ>>& controlPoints, double step) {
//	pcl::PointXYZ interpolatePoints;
//	PointCloud::Ptr interpolateCloud(new PointCloud);
//	int i = 3;
//	int m = 
//	vector<double> u1(m + 1 + 2 * k, 0);
//	double u = 0; //original code
//
//	//double u = u_s;//SIGN1
//	//these are original codes, the u ranges from 0 to 1
//	//while (u < 1) //the original code
//	for (double u = 0; u <= 1.0f; u += step) {
//		for (double v = 0; v <= 1.0f; v += step) {
//			pcl::PointXYZ point = bs(controlPoints, u, v); // Evaluate B-spline surface at (u, v)
//			interpolateCloud->push_back(point);			
//		}
//
//	while (u <= 1) { // SIGN1
//		u1[i] = u;
//		vector<double> tmp(2, 0);
//		if (u > U[i + 1])
//			i++;
//		for (int j = 0; j <= 3; j++) {
//			double nu = Nu(i - j, u, 3, U);
//			double t_x = ctrlPoints[i - j][0] * nu;
//			double t_y = ctrlPoints[i - j][1] * nu;
//			tmp[0] += t_x;
//			tmp[1] += t_y;
//		}
//		interpolatePoints.push_back(tmp);
//		u += 1.0 / (m * 20);
//
//	}
//
//	//interpolatePoints.push_back({ originPoints[m][0], originPoints[m][1] });
//	
//
//	return cloud_out;
//}

void saveInterpolatedData(const vector<vector<pcl::PointXYZ>>& controlPts, const std::string& filename) {
	std::ofstream outFile(filename);

	if (!outFile.is_open()) {
		std::cerr << "Error: Unable to open file for writing: " << filename << std::endl;
		return;
	}

	// 遍历二维向量 controlPts 并保存点数据
	for (const auto& row : controlPts) {
		for (const auto& point : row) {
			outFile << point.x << " " << point.y << " " << point.z << "\n";
		}
	}

	outFile.close();
	std::cout << "Interpolated data saved to " << filename << std::endl;
}

void visualize(PointCloud::Ptr cloud)
{
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler(cloud, 255, 255, 255);
	viewer.addPointCloud(cloud, cloud_handler, "original_cloud");
	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0, 0, 0, 0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}
