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

    // reduce the resolution/density of the point cloud
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    //std::cout << "voxel cloud " << cloud_filtered->points.size() << std::endl;

    // restrict points to the road immediately in front and behind our car
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped_filtered (new pcl::PointCloud<PointT> ());
    pcl::CropBox<PointT> cropbox(true);
    cropbox.setMin(minPoint);
    cropbox.setMax(maxPoint);
    cropbox.setInputCloud(cloud_filtered);
    cropbox.filter(*cloud_cropped_filtered);
    //std::cout << "points " << minPoint << " " << maxPoint << std::endl;
    //std::cout << "cropped cloud " << cloud_cropped_filtered->points.size() << std::endl;

 
    // filter out any points that might be reflections or our car's roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    cropbox.setMin(Eigen::Vector4f (-1.5, -1.7, -1.0, 1.0));
    cropbox.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1.0));
    cropbox.setInputCloud(cloud_cropped_filtered);
    cropbox.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds resulting in " << cloud_cropped_filtered->points.size() << " points." << std::endl;

    return cloud_cropped_filtered;
}


/***************************************************************
 * Apply Ransac algorithm to separate a cloud of points into 2
 * new clouds, one inlier cloud and one outlier cloud.  The inliers
 * are the most points that are of a given distance to a plane.
 * Return the 2 clouds as a pair:  first is outlier cloud,
 * second is inlier cloud.
 * Arguments
 * cloud is the original cloud to be separated
 * maxIterations is number of random planes to test
 * distanceThreshold is how close inlier points need to be to the plane
 ***************************************************************/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process 
    auto startTime = std::chrono::steady_clock::now();

    // use ransac to determine indices of inlier points
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;
	srand(time(NULL));

    int num_pts = cloud->points.size();
	for(int i =0;i<maxIterations;i++)
	{
        std::unordered_set<int> inliers;
		std::unordered_set<int> pt_indices;

		while(pt_indices.size() < 3)
		{
			int index = rand() % num_pts;
			pt_indices.insert(index);
		}

		auto itr = pt_indices.begin();
		PointT pt1 = cloud->points[*itr];
		itr++;
		PointT pt2 = cloud->points[*itr];
		itr++;
		PointT pt3 = cloud->points[*itr];

		float A = (pt2.y-pt1.y)*(pt3.z-pt1.z) - (pt2.z-pt1.z)*(pt3.y-pt1.y);
		float B = (pt2.z-pt1.z)*(pt3.x-pt1.x) - (pt2.x-pt1.x)*(pt3.z-pt1.z);
		float C = (pt2.x-pt1.x)*(pt3.y-pt1.y) - (pt2.y-pt1.y)*(pt3.x-pt1.x);
		float D = -(A*pt1.x+B*pt1.y+C*pt1.z);    
 
		for(int j=0;j< num_pts;j++)
		{
			PointT pt = cloud->points[j];
			float dist = fabs(A*pt.x + B*pt.y + C*pt.z + D) / sqrt(A*A + B*B + C*C);   //∣A∗x+B∗y+C∗z+D∣
			if(dist <= distanceThreshold)
			{
				inliers.insert(j);
			}
		}
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

    // split original cloud points into inlier/road/plane points and outlier/obstacle points
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds - ";
    std::cout << planeCloud->points.size() << " plane points and " << obstCloud->points.size() << " obstacle points." << std::endl;

    // return results as pair of clouds - first cloud is obstacle/outlier points, second cloud is inlier/road points
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT> 
void ProcessPointClouds<PointT>::proximity(std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, int id, int * indexes, float distanceTol)
{
	indexes[id] = 1;
	cluster.push_back(id);
	PointT point = cloud->points[id];
	std::vector<int> nearbyPointIds = tree->search(point, distanceTol);
	for(int nearbyPointId : nearbyPointIds)
	{
		if(indexes[nearbyPointId] == 0)
		{
			proximity(cluster, cloud, tree, nearbyPointId, indexes, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	int indexes[cloud->points.size()] = {};
	for(int i=0;i<cloud->points.size();i++)
	{
		if(indexes[i] == 0)
		{
			std::vector<int> cluster;
			proximity(cluster, cloud, tree, i, indexes, distanceTol);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // create and populate KdTree
    KdTree<PointT> *tree = new KdTree<PointT>;
    for (int i=0; i<cloud->points.size(); i++) 
    {
        PointT point = cloud->points[i];
    	tree->insert(point,i); 
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  	for(std::vector<int> cluster : cluster_indices)
  	{
        if(cluster.size() > minSize && cluster.size() < maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int index: cluster)
                clusterCloud->points.push_back(cloud->points[index]);
            clusters.push_back(clusterCloud);
        }
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << cluster_indices.size() << " and kept " << clusters.size() << " clusters." << std::endl;

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