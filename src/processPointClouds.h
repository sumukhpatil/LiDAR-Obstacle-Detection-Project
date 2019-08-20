// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "render/render.h"

struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id) {
		//check if tree is empty
		if(*node==NULL)
			*node = new Node(point, id);
		else
		{
			//Calculate current dim
			uint currentDepth = depth % 3;
			if (currentDepth == 0) {
				if(point.x < ((*node) -> point.x))
					insertHelper(&((*node)->left), depth + 1, point, id);
				else
					insertHelper(&((*node)->right), depth + 1, point, id);
			}	else if(currentDepth == 1) {
				if(point.y < ((*node) -> point.y))
					insertHelper(&((*node)->left), depth + 1, point, id);
				else
					insertHelper(&((*node)->right), depth + 1, point, id);
			}	else {
				if(point.z < ((*node) -> point.z))
					insertHelper(&((*node)->left), depth + 1, point, id);
				else
					insertHelper(&((*node)->right), depth + 1, point, id);
			}

		}

	}
	void insert(pcl::PointXYZI point, int id)	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)	{
		if(node != NULL) {
			if((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) && (node->point.y >= (target.y-distanceTol) && node->point.y <= (target.y + distanceTol))) {
				float dist = sqrt((node->point.x-target.x) * (node->point.x-target.x) + (node->point.y-target.y) * (node->point.y-target.y));
				if(dist <= distanceTol)
				ids.push_back(node->id);
			}

			if(depth % 3 == 0) {
				if((target.x - distanceTol) < node->point.x)
					searchHelper(target, node->left , depth + 1, distanceTol,ids);
				if((target.x + distanceTol) > node->point.x)
					searchHelper(target, node->right, depth+1, distanceTol,ids);
			}	else if(depth % 3 == 1)	{
				if((target.y - distanceTol) < node->point.y)
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				if((target.y + distanceTol) > node->point.y)
					searchHelper(target, node->right, depth+1, distanceTol, ids);
			}	else {
				if((target.z - distanceTol) < node->point.z)
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				if((target.z + distanceTol) > node->point.z)
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}

		}
	}
	std::vector<int> search(pcl::PointXYZI target, float distanceTol) {
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;

	}

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& isProcessed, KdTree* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, float minSize, float maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};
#endif /* PROCESSPOINTCLOUDS_H_ */
