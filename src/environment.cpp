

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


// tunable parameters governing cloud processing pipeline
float FILTER_RESOLUTION = 0.25;
Eigen::Vector4f CLIP_BOX_MIN_PT(-20, -6, -3, 1);
Eigen::Vector4f CLIP_BOX_MAX_PT( 20, 6, 2, 1);
int RANSAC_MAX_ITERATIONS = 100;
float RANSAC_DIST_THRESHOLD = 0.2;
float OBJECT_CLUSTERING_TOLERANCE = 0.5;
int OBJECT_MIN_POINTS = 10; 
int OBJECT_MAX_POINTS = 500;
Color PLANE_COLOR(0, 1, 0);
Color OBJECT_COLOR(0, 0, 1);


// process one scan from lidar with filtering, segmenting, clustering, and boxing, and render results in viewer
void process_cloud(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // filter raw input into voxel grid and clip uninteresting parts of scan
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, FILTER_RESOLUTION, CLIP_BOX_MIN_PT, CLIP_BOX_MAX_PT);

    // segment filtered cloud into road points cloud and obstacles points cloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, RANSAC_MAX_ITERATIONS, RANSAC_DIST_THRESHOLD);
    
    // cluster the obstacle points into objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, OBJECT_CLUSTERING_TOLERANCE, OBJECT_MIN_POINTS, OBJECT_MAX_POINTS);

    // render the road plane points
    renderPointCloud(viewer,segmentCloud.second,"planeCloud", PLANE_COLOR);

    // place boxes around objects and render
    int clusterId = 0;
    std::cout << "cluster size ";
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << cluster->points.size() << ", ";
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), OBJECT_COLOR);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    std::cout << std::endl; // ends cluster sizes line above

    std::cout << std::endl; // blank line
}


// read in a sequence of pcd files, identify the objects in each, and render results to screen
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        process_cloud(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}