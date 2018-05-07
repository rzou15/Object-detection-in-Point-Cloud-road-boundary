#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/don.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;

std::vector<std::vector<std::string>> 
ReadDataFromCSV(std::string filename)
{
	std::ifstream file(filename);
	std::vector<std::vector<std::string> > dataList;

	std::string line = "";
	while (getline(file, line))
	{
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
		dataList.push_back(vec);
	}
	return dataList;
}

void
pcdWriter(std::vector<std::vector<std::string>> rawData, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
	//pcl::PointCloud<pcl::PointXYZI> cloud1;

	// Fill in the cloud data
	cloud->width = (uint32_t) rawData.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		//std::string::size_type sz;     // alias of size_t
		cloud->points[i].x = std::stof(rawData[i][0]);
		cloud->points[i].y = std::stof(rawData[i][1]);
		cloud->points[i].z = std::stof(rawData[i][2]);
		cloud->points[i].intensity = std::stof(rawData[i][3]);
	}

	pcl::io::savePCDFileASCII("final_project_point_cloud.pcd", *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to final_project_point_cloud.pcd file." << std::endl;
	 
	//for (size_t i = 0; i < cloud->points.size(); ++i)
	//	std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
}

void
DoNWriter(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, std::string fileName) {
	std::vector<float> mag;
	std::ofstream out(fileName);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		//std::string::size_type sz;     // alias of size_t
		mag.push_back(cloud->points[i].curvature);
	}
	std::ostream_iterator<float> out_iterator(out, "\n");
	std::copy(mag.begin(), mag.end(), out_iterator);
	out.close();

	//for (size_t i = 0; i < cloud->points.size(); ++i)
	//	std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
}

void
cloudGrandRemover(pcl::PointCloud<pcl::PointNormal>::Ptr &originCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*originCloud, *cloud);

	std::cout << "Remove grand plane..." << std::endl;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);

	// Get the points associated with the planar surface
	extract.filter(*cloud_seg);
	std::cout << "PointCloud representing the planar component: " << cloud_seg->points.size() << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_filtered, *originCloud);
}

void
DiffNormalSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segCloud) {	
	double scale1 = 0.06;//The smallest scale to use in the DoN filter.	
	double scale2 = 0.6;//The largest scale to use in the DoN filter.	
	double threshold = 0.5;//The minimum DoN magnitude to threshold by
	double segradius = 0.6;//segment scene into clusters with given distance tolerance using euclidean clustering

	// Create a search tree, use KDTreee for non-organized data.
	std::cout << "creating search tree using KDTree" << std::endl;
	pcl::search::Search<pcl::PointXYZI>::Ptr tree;
	if (cloud->isOrganized())
	{
		tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZI>());
	}
	else
	{
		tree.reset(new pcl::search::KdTree<pcl::PointXYZI>(false));
	}

	// Set the input pointcloud for the search tree
	tree->setInputCloud(cloud);

	if (scale1 >= scale2)
	{
		cerr << "Error: Large scale must be > small scale!" << endl;
		exit(EXIT_FAILURE);
	}

	//********************************************DoN***********************************************************************
	// Compute normals using both small and large scales at each point
	pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointNormal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setNumberOfThreads(7);

	/**
	* NOTE: setting viewpoint is very important, so that we can ensure
	* normals are all pointed in the same direction!
	*/
	//ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	ne.setViewPoint(-50,-50,-50);

	// calculate normals with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch(scale1);
	ne.compute(*normals_small_scale);

	// calculate normals with the large scale
	cout << "Calculating normals for scale..." << scale2 << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch(scale2);
	ne.compute(*normals_large_scale);

	// Create output cloud for DoN results
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZI, pcl::PointNormal>(*cloud, *doncloud);
	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Compute DoN
	don.computeFeature(*doncloud);

	// Save DoN features
	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);
	DoNWriter(doncloud,"MagDoN.txt");
	//********************************************End of DoN***********************************************************************

	//********************************************Filtering************************************************************************
	// Filter by magnitude
	cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

	// Build the condition for filtering
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(
		new pcl::ConditionOr<pcl::PointNormal>()
	);
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
		new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold))
	);
	// Build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
	//pcl::PointIndices::Ptr donOutliers(new pcl::PointIndices);
	condrem.setInputCloud(doncloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	condrem.filter(*doncloud_filtered);
	//donOutliers = condrem.getRemovedIndices();

	doncloud = doncloud_filtered;

	//*******************************************************************************************************************************
	cloudGrandRemover(doncloud);
	std::cout << "Plane removed Pointcloud: " << doncloud->points.size() << " data points." << std::endl;
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZRGBA>(*doncloud, *segCloud);
	//*******************************************************************************************************************************

	// Save filtered output
	std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;

	writer.write<pcl::PointNormal>("don_filtered.pcd", *doncloud, false);
	//********************************************End of Filtering*******************************************************************

	//********************************************Clustering*************************************************************************
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr doncloud_intermediate(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZRGBA>(*doncloud, *doncloud_intermediate);

	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr segtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	segtree->setInputCloud(doncloud_intermediate);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;

	ec.setClusterTolerance(segradius);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(40000);
	ec.setSearchMethod(segtree);
	
	ec.setInputCloud(doncloud_intermediate);//doncloud
	ec.extract(cluster_indices);

	std::cout << "Segmenting point cloud and add colors..." << std::endl;
	
	int j = 0;
	uint8_t red = 255;
	uint8_t green = 0;
	uint8_t blue = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointNormal>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster_don->points.push_back(doncloud->points[*pit]);
			segCloud->at(*pit).r = red;
			segCloud->at(*pit).g = green;
			segCloud->at(*pit).b = blue;
			segCloud->at(*pit).a = 255;
		}
		red = rand() % 256;
		green = rand() % 256;
		blue = rand() % 256;

		cloud_cluster_don->width = int(cloud_cluster_don->points.size());
		cloud_cluster_don->height = 1;
		cloud_cluster_don->is_dense = true;

		//Save cluster
		cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "don_cluster_" << j << ".pcd";
		writer.write<pcl::PointNormal>(ss.str(), *cloud_cluster_don, false);
		ss << "Mag_don_cluster_" << j << ".txt";
		DoNWriter(cloud_cluster_don, ss.str());
	}
	writer.write<pcl::PointXYZRGBA>("segCould.pcd", *segCloud, false);
}

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.1, 1.0, 0.7);
	std::vector<std::vector<std::string>> trajectory = ReadDataFromCSV("trajectory_camera_coord.csv");
	for (int i = 0; i < trajectory.size(); ++i) {
		pcl::PointXYZ o;
		o.x = std::stof(trajectory[i][0]);
		o.y = std::stof(trajectory[i][1]);
		o.z = std::stof(trajectory[i][2]);
		viewer.addSphere(o, 10, "sphere", 0);
	}
	std::cout << "this only runs once" << std::endl;

}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int
main()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//std::cout << "Reading data from csv file..." << std::endl;
	//std::vector<std::vector<std::string> > raw_data = ReadDataFromCSV("point_cloud_camera_coord.csv");
	//std::cout << "Writing data into pcd file..." << std::endl;
	//pcdWriter(raw_data, cloud);
	//raw_data.clear();

	std::string fileName = "final_project_point_cloud.pcd";
	std::cout << "Loading point cloud from file: " << fileName << std::endl;
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(fileName, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	DiffNormalSegmentation(cloud, segCloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZRGBA>(*cloud, *finalCloud);
	*finalCloud += *segCloud;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGBA>("final.pcd", *finalCloud, false);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//blocks until the cloud is actually rendered
	viewer.showCloud(finalCloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	return 0;
}