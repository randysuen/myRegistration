#include <iostream>
#include <string>
//boost headers
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include "defines.h"
#include "utils.h"
#include "operations.h"
#include <vector>
#include <string>

//初始化Utils类的静态成员
bool Utils::pcd_ex = false;
bool Utils::ply_ex = false;



int main(){
	std::string pDirPath = "data";
	std::vector<std::string> pFileNames;
	Utils::get_clouds_filenames(pDirPath, pFileNames);
	bool use_pcd = Utils::pcd_ex;
	bool use_ply = Utils::ply_ex;
	std::cout << "use_pcd:" << use_pcd << std::endl;
	std::cout << "use_ply:" << use_ply << std::endl;


	pcl::PointCloud<myPointT>::Ptr currentCloud(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myNormalT>::Ptr currentNormals(new pcl::PointCloud<myNormalT>);
	pcl::PointCloud<myPointT>::Ptr previousCloud(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myNormalT>::Ptr previousNormals(new pcl::PointCloud<myNormalT>);
	pcl::PointCloud<myNormalT>::Ptr normals(new pcl::PointCloud<myNormalT>);
	pcl::PointCloud<myPointT>::Ptr currentKeypoints(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myPointT>::Ptr previousKeypoints(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<pcl::SHOT1344>::Ptr currentDescriptors(new pcl::PointCloud<pcl::SHOT1344>);
	pcl::PointCloud<pcl::SHOT1344>::Ptr previousDescriptors(new pcl::PointCloud<pcl::SHOT1344>);
	pcl::PointCloud<myPointT>::Ptr previousCloudTrans(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myPointT>::Ptr previousKeypointsTrans(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myPointT>::Ptr currentKeypointsTrans(new pcl::PointCloud<myPointT>);

	pcl::CorrespondencesPtr finalCorrespondences(new pcl::Correspondences);
	pcl::PointCloud<myPointT>::Ptr currentInitAlinedCloud(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myPointT>::Ptr currentFinalAlinedCloud(new pcl::PointCloud<myPointT>);
	pcl::PointCloud<myPointT>::Ptr concatenateCloud(new pcl::PointCloud<myPointT>);

	Eigen::Matrix4f initTransMatrix;
	Eigen::Matrix4f icpTransMatrix;
	Eigen::Matrix4f finalTransMatrix;

	for (auto it = pFileNames.begin(); it != pFileNames.end(); it++){
		if (use_pcd){
			pcl::io::loadPCDFile<myPointT>(*it, *currentCloud);
		}
		if (use_ply){
			pcl::io::loadPLYFile<myPointT>(*it, *currentCloud);
		}
		//计算当前帧的法线
		PointCloudOperations::compute_normals(currentCloud, 10, normals);

		//计算当前帧susan关键点
		PointCloudOperations::compute_susan(currentCloud, 0.005, currentKeypoints);

		//计算前帧CSHOT
		PointCloudOperations::compute_cshot(currentCloud, normals, currentKeypoints, 0.05, currentDescriptors);
		
/*		if (it == pFileNames.begin()) {
			pcl::copyPointCloud(*currentCloud, *previousCloud);
			*concatenateCloud += *currentCloud;
		}*/

//		else {
		if (it!=pFileNames.begin()){
			//后作为source,前作为target
			//对前后帧描述子进行匹配
			PointCloudOperations::find_correspondences(currentDescriptors, previousDescriptors, currentKeypoints, previousKeypoints,  0.005, finalCorrespondences);
			std::cout << "checkpoint2" << std::endl;
			//计算前后帧的初始转移矩阵
			PointCloudOperations::get_initTransMatrix(currentKeypoints, previousKeypoints, finalCorrespondences, initTransMatrix);
			//前后帧进行粗配准
			pcl::transformPointCloud(*currentCloud, *currentInitAlinedCloud, initTransMatrix);
			//前后帧进行ICP细配准			
			PointCloudOperations::icp_align_cloud(currentInitAlinedCloud, previousCloud, currentFinalAlinedCloud, 100, 0.005, icpTransMatrix);
			//计算最终的转移矩阵
			//finalTransMatrix = initTransMatrix*icpTransMatrix;
			//计算最终配准的当前帧的关键点和描述子
		//	PointCloudOperations::compute_normals(currentFinalAlinedCloud, 10, normals);
		//	PointCloudOperations::compute_susan(currentFinalAlinedCloud, 0.01, currentKeypoints);
		//	PointCloudOperations::compute_cshot(currentFinalAlinedCloud, normals, currentKeypoints, 0.05, currentDescriptors);

		//	pcl::copyPointCloud(*currentFinalAlinedCloud, *previousCloud);
			*concatenateCloud += *currentFinalAlinedCloud;

		}

		//将最终配准的当前帧融合到一起
		if (it == pFileNames.begin()) {
			pcl::copyPointCloud(*currentCloud, *previousCloud);
			std::cout << "chpppp" << std::endl;
			pcl::copyPointCloud(*currentKeypoints, *previousKeypoints);
			pcl::copyPointCloud(*currentDescriptors, *previousDescriptors);
		}
        //把最终配准的当前帧存到previousCloud里
/*		if (it == pFileNames.begin()){

			pcl::copyPointCloud(*currentCloud, *previousCloud);
			pcl::copyPointCloud(*currentKeypoints, *previousKeypoints);
			pcl::copyPointCloud(*currentDescriptors, *previousDescriptors);

		}*/
		std::cout << "curr size :" << currentCloud->size() << std::endl;
		std::cout << "previous size" << previousCloud->size() << std::endl;
	}
	pcl::visualization::PCLVisualizer viewer("display");
//	int vp1, vp2;
//	viewer.createViewPort(0, 0, 0.5, 1, vp1);
//	viewer.createViewPort(0.5, 0, 1, 1, vp2);


/*	viewer.addPointCloud(currentCloud, "current cloud", vp1);
	viewer.addPointCloud(previousCloud, "previous cloud", vp1);
	viewer.addPointCloud(concatenateCloud, "current aligned cloud", vp2);*/
//	viewer.addPointCloud(previousCloud, "previous cloud", vp2);
//	pcl::visualization::PointCloudColorHandlerCustom<myPointT> keypoints_handler(currentKeypoints, 0, 0, 255);
//	viewer.addPointCloud(currentKeypoints, keypoints_handler, "currentkeypoints", vp1);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "currentkeypoints");
//	viewer.addPointCloud(previousKeypoints, keypoints_handler, "previouskeypoints", vp2);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "previouskeypoints");
	
//	pcl::visualization::PCLVisualizer viewer("display");
	//为了在同一个窗口显示前后帧，并且画出匹配线，需要将前帧平移一下
	pcl::transformPointCloud(*previousCloud, *previousCloudTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*previousKeypoints, *previousKeypointsTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	viewer.addPointCloud(currentCloud, "current cloud");
	viewer.addPointCloud(previousCloudTrans, "previous cloud");

	pcl::visualization::PointCloudColorHandlerCustom<myPointT> keypoints_handler(currentKeypoints, 0, 0, 255);

	viewer.addPointCloud(currentKeypoints, keypoints_handler, "currentkeypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "currentkeypoints");
	viewer.addPointCloud(previousKeypointsTrans, keypoints_handler, "previouskeypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "previouskeypoints");
	
	for (size_t i = 0; i < finalCorrespondences->size(); i++) {
		std::stringstream line_ID;
		line_ID << "correspondence_line_" << i;	
		viewer.addLine<myPointT, myPointT>(currentKeypoints->at(finalCorrespondences->at(i).index_query), previousKeypointsTrans->at(finalCorrespondences->at(i).index_match), 0, 255, 0, line_ID.str());
	}
	std::cout << "checkpoint" << std::endl;
	
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return 0;
	
}