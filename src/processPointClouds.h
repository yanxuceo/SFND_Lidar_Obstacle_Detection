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



template<typename PointT>
class ProcessPointClouds {
    // Structure to represent node of kd tree
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper3D(Node** node, uint depth, PointT point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			uint cd = depth % 3;

            if(cd == 0)
            {
                if(point.x < ((*node)->point.x))
                {
                    insertHelper3D(&((*node)->left), depth+1, point, id);
                }
			    else
			    {
				    insertHelper3D(&((*node)->right), depth+1, point, id);
			    }
            }
            else if (cd == 1)
            {
                if(point.y < ((*node)->point.y))
                {
                    insertHelper3D(&((*node)->left), depth+1, point, id);
                }
			    else
			    {
				    insertHelper3D(&((*node)->right), depth+1, point, id);
			    }
            }
            else
            {
                if(point.z < ((*node)->point.z))
                {
                    insertHelper3D(&((*node)->left), depth+1, point, id);
                }
			    else
			    {
				    insertHelper3D(&((*node)->right), depth+1, point, id);
			    }
            }
		}
	}

	void insert3D(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper3D(&root, 0, point, id);
	}

	void searchHelper3D(PointT target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if( (node->point.x >= (target.x-distanceTol) && node->point.x <= (target.x+distanceTol)) && (node->point.y>=(target.y-distanceTol) && node->point.y <= (target.y+distanceTol)) && (node->point.z>=(target.z-distanceTol) && node->point.z <= (target.z+distanceTol)))
			{
				float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) +  (node->point.y - target.y)*(node->point.y - target.y) + (node->point.z - target.z)*(node->point.z - target.z));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

            uint cd = depth % 3;
            if (cd == 0)
            {
                if((target.x-distanceTol) < node->point.x)
				    searchHelper3D(target, node->left, depth+1, distanceTol, ids);
			    if((target.x+distanceTol) > node->point.x)
				    searchHelper3D(target, node->right, depth+1, distanceTol, ids); 
            }
            else if(cd == 1)
            {
                if((target.y-distanceTol) < node->point.y)
				    searchHelper3D(target, node->left, depth+1, distanceTol, ids);
			    if((target.y+distanceTol) > node->point.y)
				    searchHelper3D(target, node->right, depth+1, distanceTol, ids); 
            }
            else
            {
                if((target.z-distanceTol) < node->point.z)
				    searchHelper3D(target, node->left, depth+1, distanceTol, ids);
			    if((target.z+distanceTol) > node->point.z)
				    searchHelper3D(target, node->right, depth+1, distanceTol, ids); 
            }
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper3D(target, root, 0, distanceTol, ids);

		return ids;
	}
};



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

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

private:
    KdTree* tree;

    void clusterHelper3D(int indice, typename pcl::PointCloud<PointT>::Ptr points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
    {
	    processed[indice] = true;
	    cluster.push_back(indice);

	    std::vector<int> nearest = tree->search3D(points->points[indice], distanceTol);

	    for(int id : nearest)
	    {
		    if(!processed[id])
			    clusterHelper3D(id, points, cluster, processed, tree, distanceTol);
	    }
    }
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */