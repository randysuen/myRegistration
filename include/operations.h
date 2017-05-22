#ifndef POINTCLOUDOPERATIONS_HPP_
#define POINTCLOUDOPERATIONS_HPP_

#include <iostream>
#include <string>
#include "defines.h"

// PCL input/output
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/registration/icp.h>

// PCL features
#include <pcl/keypoints/susan.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot_omp.h>

// PCL registration
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

// PCL filtering
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
// PCL clustering
#include <pcl/segmentation/extract_clusters.h>
// PCL surface reconstruction
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
// PCL visualization
#include <pcl/visualization/pcl_visualizer.h>

#define DEBUG 1

class PointCloudOperations
{
public:

	static void filter_bilateral
		(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & pSrcCloud,
		const float & pSigmaR,
		const float & pSigmaS,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pDstCloud
		);

	static void compute_normals
		(
		const pcl::PointCloud<myPointT>::ConstPtr & pCloud,
		const int & pNeighbors,
		//const float & pRadius,
			pcl::PointCloud<myNormalT>::Ptr & pNormals
		);

	static void ror_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const int & pNeighbors,
		const float & pRadius
		);

	static void sor_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const int & pMeanK,
		const float & pStdDev
		);

	static void vg_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const float & pLeafSize
		);

	static void ece_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const float & pClusterTolerance,
		const int & pMinClusterSize,
		const int & pMaxClusterSize,
		const int & pClusters
		);

	static void translate_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const Eigen::Vector3f & pTranslation
		);

	static void rotate_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const double & pAngle,
		const Eigen::Vector3f & pAxis
		);

	static void concatenate_clouds
		(
		const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & pClouds,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud
		);

	static void pmr_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PolygonMesh & pDstMesh,
		const float & pMlsSearchRadius,
		const bool & pMlsPolynomialFit,
		const int & pMlsPolynomialOrder,
		const float & pMlsUpsamplingRadius,
		const float & pMlsUpsamplingStepSize,
		const float & pNeRadiusSearch,
		const int & pPoissonDepth
		);

	static void icp_align_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pTgtCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const int & pMaxIterations,
		const double & pEpsilon,
			Eigen::Matrix4f & icpTransMatrix

		);

	static void icp_normals_align_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pTgtCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const int & pMaxIterations,
		const double & pEpsilon,
			Eigen::Matrix4f & icpTransMatrix
		);

	static void cut_cloud
		(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pSrcCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pDstCloud,
		const int & pAmount
		);
	

	static void PointCloudOperations::compute_cshot
		(
			const pcl::PointCloud<myPointT>::Ptr & pScrCloud,
			const pcl::PointCloud<pcl::Normal>::Ptr & pNormals,
			const pcl::PointCloud<myPointT>::Ptr & kpCloud,
			//const int & pNeighbors
			const float & pRadius,
			pcl::PointCloud<pcl::SHOT1344>::Ptr & cshotDescriptor
			);
	static void PointCloudOperations::compute_susan(
		const pcl::PointCloud<myPointT>::ConstPtr & pSrcCloud,
//		const pcl::PointCloud<myNormalT>::ConstPtr & pNormals,
		const float & pRadius,
		pcl::PointCloud<myPointT>::Ptr & pDstCloud

		);
	static void PointCloudOperations::find_correspondences(
		const pcl::PointCloud<pcl::SHOT1344>::ConstPtr & srcDescriptors,
		const pcl::PointCloud<pcl::SHOT1344>::ConstPtr & tarDescriptors,
		const pcl::PointCloud<myPointT>::ConstPtr & srcKeypoints,
		const pcl::PointCloud<myPointT>::ConstPtr & tarKeypoints,
		const float & mathThreshold,
		pcl::CorrespondencesPtr & finalCorrespondences
		);

	static void PointCloudOperations::get_initTransMatrix(
		const pcl::PointCloud<myPointT>::ConstPtr & srcKeypoints,
		const pcl::PointCloud<myPointT>::ConstPtr & tarKeypoints,
		const pcl::CorrespondencesPtr & correspondences,
		Eigen::Matrix4f & transMatrix
		);
};

#endif