// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(filterRes, filterRes, filterRes);
    voxel.filter(*filteredCloud);

    typename pcl::CropBox<PointT> cropBox(true);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for (int point : indices) {
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
  auto start = std::chrono::steady_clock::now();
	// For max iterations
	while (maxIterations) {
		std::unordered_set<int> inliers;
	// Randomly sample subset and fit line
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}
	float x1, y1, z1, x2, y2, z2, x3, y3, z3;
	auto itr = inliers.begin();
	x1 = cloud->points[*itr].x;
	y1 = cloud->points[*itr].y;
	z1 = cloud->points[*itr].z;
	itr++;
	x2 = cloud->points[*itr].x;
	y2 = cloud->points[*itr].y;
	z2 = cloud->points[*itr].z;
	itr++;
	x3 = cloud->points[*itr].x;
	y3 = cloud->points[*itr].y;
	z3 = cloud->points[*itr].z;

	float a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
	float b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
	float c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
	float d = -((x1 * (((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)))) + (y1 * (((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)))) + (z1 * (((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)))));

	for (int index = 0; index < cloud->points.size(); index++) {
		float x4, y4, z4;
		x4 = cloud->points[index].x;
		y4 = cloud->points[index].y;
		z4 = cloud->points[index].z;
		if (inliers.count(index) > 0) {
			continue;
		}
		// Measure distance between every point and fitted line
		float dist = fabs((a * x4 + b * y4 + c * z4 + d))/sqrt(a * a + b * b + c * c);
// If distance is smaller than threshold count it as inlier
		if (dist <= distanceTol) {
			inliers.insert(index);
		}
	}
	if (inliers.size() > inliersResult.size()) {
		inliersResult = inliers;
	}
	maxIterations--;
}
auto end = std::chrono::steady_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
std::cout << "RANSAC took " << duration.count() << " milliseconds\n";
	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster_id, std::vector<bool>& isProcessed, KdTree* tree, float distanceTol) {
	isProcessed[index] = true;
	cluster_id.push_back(index);
	std::vector<int> nearbyPoints = tree->search(cloud->points[index], distanceTol);
	for (int id : nearbyPoints) {
		if (!isProcessed[id]) {
			proximity(id, cloud, cluster_id, isProcessed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, float minSize, float maxSize) {
  auto start = std::chrono::steady_clock::now();
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> isProcessed(cloud->points.size(), false);
	int i = 0;
	while (i < cloud->points.size()) {
		if (isProcessed[i]) {
			i++;
			continue;
		}
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    std::vector<int> cluster_id;
		proximity(i, cloud, cluster_id, isProcessed, tree, distanceTol);
    if (cluster_id.size() < maxSize && cluster_id.size() > minSize) {
      for (int i : cluster_id) {
        cluster->points.push_back(cloud->points[i]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
      }
      clusters.push_back(cluster);
  		}
		i++;
	}
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Euclidean Clustering took " << duration.count() << " milliseconds\n";

	return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
