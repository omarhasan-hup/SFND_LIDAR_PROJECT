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
  /*creating the new object to downsize the point cloud data to 
  the dimentions of the box given to it*/
   pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT> ::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (filterRes,filterRes ,filterRes);
  vg.filter (*cloudFiltered);
  /*after creating the filltered cloud then we will create its box to be able 
    to render it*/
  typename pcl::PointCloud<PointT> ::Ptr cloudRegion (new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT>region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud (cloudFiltered);
  region.filter (*cloudRegion);
  
  /*now we make a for loop to extraxt the roof points from the point cloud  */
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
  roof.setMax(Eigen::Vector4f (2.6,1.7,-4,1));
  roof.setInputCloud (cloudRegion);
  roof.filter (indices);
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  for(int point :indices)
     inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloudRegion);


  

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT> ::Ptr obstCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT> ::Ptr planeCloud (new pcl::PointCloud<PointT> ());
  //after creating two different points one for the obstacle and the other for the plane points
  //we shall now extract the points to put them in the right inliers
  for(int index: inliers->indices)
      planeCloud->points.push_back(cloud->points[index]);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative (true);
  extract.filter(*obstCloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);


    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    
    pcl::ModelCoefficients::Ptr cofficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;
    //make an object of the class pointT which is actually a template
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment (*inliers,*cofficients);
    if(inliers->indices.size()==0)
    {
        std::cerr <<"colud not estimate a planner model for the given datasheet"<< std::endl;
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (clusterTolerance);
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (clusterIndices);
  for(pcl::PointIndices getIndices :clusterIndices)
  {
   typename pcl::PointCloud<PointT>:: Ptr cloudCluster (new pcl::PointCloud<PointT>);
    for(int index :getIndices.indices)
    {
        cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
     
  }
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
/*my functions are blow here */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) 
{
  auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function


	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  /*at first wi will make a loop to check the iterations is grater than zero 
    then we will insert three random points from the data to form a plane using the given plane equation
    NOTE: we must check if the the inserted three points are the same or not 
    after that we will coninue */
  while(maxIterations--)
  {
    
    /*entering this loop here means the iterations is bigger than zero so we shall begin and insert the three different points to try to fit a plane between them using the given plane function   */
    std::unordered_set <int> inliers;
    /*this line garantes you a set of int values of your points */
    while(inliers.size() < 3)
    {
      /*entering this loop means the inliers size is less than three. 
        this means there are less than three points 
        so we need to insert another random point utill we reach three*/
      inliers.insert(rand()%(cloud->points.size()));
      /*the % btweeen the rand() function and the (cloud->points.size())
        this means we can insert a point from 0 to the number of points
        inside the cloud.
        also the the unordered_set menas that there are no repeation here*/
    }
    float x1,y1,z1,x2,y2,z2,x3,y3,z3;//create 9 float varible to store the three points
    auto itr=inliers.begin();
    x1=cloud->points[*itr].x;
    y1=cloud->points[*itr].y;
    z1=cloud->points[*itr].z;
    itr++;
    x2=cloud->points[*itr].x;
    y2=cloud->points[*itr].y;
    z2=cloud->points[*itr].z;
    itr++;
    x3=cloud->points[*itr].x;
    y3=cloud->points[*itr].y;
    z3=cloud->points[*itr].z;
    /*from line 102 to line 113 we only stored the points numbers inside 
      the 9 float variables */
    /*after this we will create two vectores of float types to travel from point 1 to the
      other two points
      v1 will travel from point 1 to point 2
      v2 will travel from point 1 to point 3*/
//     vector <float> v1(3)={(x2-x1),(y2-y1),(z2-z1)};
//     vector <float> v2(3)={(x3-x1),(y3-y1),(z3-z1)};
    
    float i=(((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1)));//i=a
    float j=(((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1)));//j=b
    float k=(((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1)));//k=c
    float D=(-((i*x1)+(j*y1)+(k*z1)));
    /*now we have the plane equation which will be as follow
      Ax+By+Kz+D=0
      we can use this equation to find out the distance from each point*/
    
    
    /*create three more float variables to store the a & b & c & d constants
      of the line equation
      and calculating there values*/
    
    /*after we got the two points which made a line, we will go inside 
      a loop to see how many points are close to that line.
      this check is done using the distace equation 
    */
    for(int index=0;index <(cloud->points.size());index++)
    {
      /*before looping on the points in the cloud, we should avoid looping
        on the points which they are forming the line.
        this is done by checking the index of the point. if that index 
        matches the line points index then skip the loop this time*/
      if(inliers.count(index) > 0)
      {
        continue;
      }
      /*so if the index is differant from the line points index
        then we store this point and calculte its distance from the line.
        if the disatnce is less the minumum distace then this point will 
        be added*/
    pcl::PointXYZ point=cloud->points[index];
    float x4 = point.x;
    float y4 = point.y;
    float z4 = point.z;  
    float d=(fabs((i*x4)+(j*y4)+(k*z4)+D))/(sqrt((i*i)+(j*j)+(k*k)));
    if (d <=distanceTol)
       {
         inliers.insert(index);
       }
    }
   if(inliers.size() > inliersResult.size())
     {
       inliersResult=inliers;
     }
  }
	
// 	return inliersResult;
  auto endTime = std::chrono::steady_clock::now();
	auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took " << ellapsedTime.count() << " millicesonds" << std::endl;

	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];

		if (inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	
	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers); 

}
