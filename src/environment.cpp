/* \author Aaron Brown and Sumukh Patil*/
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, Eigen::Vector4f(-18, -5, -5, 1), Eigen::Vector4f(18, 6, 10, 1));
  std::unordered_set<int> ransacI = pointProcessorI->Ransac(filterCloud, 100, 0.2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZI>());  // PointCloud containing the obstacles
  pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>()); // PointCloud containing the plane

  for (int index = 0; index < filterCloud->points.size(); index++) {
    pcl::PointXYZI point = filterCloud->points[index];
    if (ransacI.count(index)) {
      inliers->points.push_back(point);
    } else {
      outliers->points.push_back(point);
    }
  }
  if (ransacI.size()) {
    renderPointCloud(viewer, inliers, "Obstacle Cloud", Color(0, 1, 0));
  } else {
    renderPointCloud(viewer, filterCloud, "Filtered Cloud");
  }
  KdTree* tree = new KdTree;
  for (int i = 0; i < outliers->points.size(); i++) {
    tree->insert(outliers->points[i], i);
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusterI = pointProcessorI->euclideanCluster(outliers, tree, 0.4, 12, 275);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusterI) {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstacle cloud"+std::to_string(clusterId), colors[clusterId]);
    ++clusterId;
    Box boundingBoxI = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, boundingBoxI, clusterId);
  }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;


    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloud);
        streamIterator++;
        if (streamIterator == stream.end()) {
          streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
}
