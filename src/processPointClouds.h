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
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT pt, int setId)
	:	point(pt), id(setId), left(NULL), right(NULL)
	{}
};

// KDTree iimplementation
template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT> **node, int depth, PointT point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node<PointT>(point, id);
			return;
		} else {
			int index = depth % 3;
            depth = depth+1;
            if(index == 0)   // branch on x
            {
                if(point.x < (*node)->point.x)
                {
                    return insertHelper(&((*node)->left), depth, point, id);
                } else {
                    return insertHelper(&((*node)->right), depth, point, id);
                }
            }
            else if(index == 1)  // branch on y
            {
                if(point.y < (*node)->point.y)
			    {
				    return insertHelper(&((*node)->left), depth, point, id);
			    } else {
				    return insertHelper(&((*node)->right), depth, point, id);
			    }
            } 
            else  // branch on z
            {
                if(point.z < (*node)->point.z)
			    {   
				    return insertHelper(&((*node)->left), depth, point, id);
			    } else {
				    return insertHelper(&((*node)->right), depth, point, id);
			    }
            }
		}
	}

	void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<int> &ids, int depth, Node<PointT> *node, PointT target, float distanceTol)
	{
		if(node == NULL)
		{
			return;
		}
		if(
            (node)->point.x>=(target.x-distanceTol) && (node)->point.x<(target.x+distanceTol)
		    && 
            (node)->point.y>=(target.y-distanceTol) && (node)->point.y<(target.y+distanceTol)
            &&
            (node)->point.z>=(target.z-distanceTol) && (node)->point.z<(target.z+distanceTol))
		   {
			   float x_dist = (node)->point.x - target.x;
			   float y_dist = (node)->point.y - target.y;
               float z_dist = (node)->point.z - target.z;
			   float dist = sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);
			   if(dist <= distanceTol)
			   {
				   ids.push_back((node)->id);
			   }
		    }
		    int index = depth % 3;
            depth += 1;
            if(index == 0)  // search on x
            {
                if(node->point.x < (target.x+distanceTol))
                {
                    searchHelper(ids, depth, node->right, target, distanceTol);
                }
                if(node->point.x > (target.x-distanceTol))
                {
                    searchHelper(ids, depth, node->left, target, distanceTol);
                }
            }
            else if(index == 1)  // search on y
            {
                if(node->point.y < (target.y+distanceTol))
                {
                    searchHelper(ids, depth, node->right, target, distanceTol);
                }
                if(node->point.y > (target.y-distanceTol))
                {
                    searchHelper(ids, depth, node->left, target, distanceTol);
                }
            }
            else  // search on z
            {
                if(node->point.z < (target.z+distanceTol))
                {
                    searchHelper(ids, depth, node->right, target, distanceTol);
                }
                if(node->point.z > (target.z-distanceTol))
                {
                    searchHelper(ids, depth, node->left, target, distanceTol);
                }
            }	   
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, 0, root, target, distanceTol);
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

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol);
    
    void proximity(std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, int id, int * indexes, float distanceTol);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */