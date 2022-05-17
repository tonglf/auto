#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

void ICP_by_SVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f &transform);
void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target);

int main(int argc, char **argv) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("../data/Reg/first.pcd", *source) != -1) {
		std::cout << "source size: " << source->size() << std::endl;
	} else {
		std::cout << "source loading failed!" << std::endl;
		return -1;
	}
	if (pcl::io::loadPCDFile("../data/Reg/second.pcd", *target) != -1) {
		std::cout << "target size: " << target->size() << std::endl;
	}
	else {
		std::cout << "target loading failed!" << std::endl;
		return -1;
	}

	pcl::VoxelGrid<pcl::PointXYZ> voxel_source;
	voxel_source.setInputCloud(source);
	voxel_source.setLeafSize(0.5, 0.5, 0.5);
	voxel_source.filter(*source);
	std::cout << "source downsample size: " << source->size() << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> voxel_target;
	voxel_target.setInputCloud(target);
	voxel_target.setLeafSize(0.5, 0.5, 0.5);
	voxel_target.filter(*target);
	std::cout << "target downsample size: " << target->size() << std::endl;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	ICP_by_SVD(source, target, transform);

	std::cout << "transform : " << std::endl << transform << std::endl;

	pcl::transformPointCloud(*source, *source, transform);

	showPointCloud(source, target);

	return 0;
}

void ICP_by_SVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f &transform) {

	if (!source || !target)
		return;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>(*source));

	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(target);

	transform.setIdentity();

	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	for (int iter = 0; iter < 35; ++iter) {
		std::cout << "iter number: " << iter + 1 << std::endl;
		Eigen::MatrixXf P(3, source->size());
		Eigen::MatrixXf Q(3, source->size());
		for (size_t i = 0; i < transform_cloud->size(); ++i) {
			std::vector<int> index;
			std::vector<float> distance;
			tree.nearestKSearch(transform_cloud->at(i), 1, index, distance);

			P(0, i) = transform_cloud->at(i).x;
			P(1, i) = transform_cloud->at(i).y;
			P(2, i) = transform_cloud->at(i).z;
			Q(0, i) = target->at(index[0]).x;
			Q(1, i) = target->at(index[0]).y;
			Q(2, i) = target->at(index[0]).z;
		}

		Eigen::Vector3f p_mean = P.rowwise().mean();
		Eigen::Vector3f q_mean = Q.rowwise().mean();
		Eigen::MatrixXf one_matrix(1, source->size());
		one_matrix.setOnes();
		P = P - p_mean * one_matrix;
		Q = Q - q_mean * one_matrix;

		Eigen::Matrix3f W = P * Q.transpose();
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		Eigen::Matrix3f R = V * U.transpose();
		if (R.determinant() < 0)
			R = -R;
		Eigen::Vector3f t = q_mean - R * p_mean;

		transform_matrix.block<3, 3>(0, 0) = R;
		transform_matrix.block<3, 1>(0, 3) = t;

		pcl::transformPointCloud(*transform_cloud, *transform_cloud, transform_matrix);

		transform = transform_matrix * transform;

		if (transform_matrix.isIdentity(1e-4))
			break;
	}
}

void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target) {

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// 添加需要显示的点云数据
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_source(source, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(source, single_color_source, "source");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_target(target, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target, single_color_target, "target");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");


	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}