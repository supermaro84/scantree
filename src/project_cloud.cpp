#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class project{
	private:
		ros::NodeHandle n;
		ros::Publisher pub_n1;
                ros::Publisher pub_n2;
		ros::Subscriber sub_n;
	public:
		project();
		void callback(const PointCloud::ConstPtr &msg)
		{
                //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		PointCloud::Ptr msg2 (new PointCloud);
		PointCloud::Ptr msg3 (new PointCloud);
                PointCloud::Ptr cloud_f (new PointCloud);
  		msg3->header.frame_id = "velodyne";
 		msg3->height = msg->height;
		msg3->width = msg->width;
                BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
 		msg3->points.push_back (pcl::PointXYZ(pt.x,pt.y,pt.z));

               
	// Create the segmentation object for the planar model and set all the parameters
		  pcl::SACSegmentation<pcl::PointXYZ> seg;
		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		  pcl::PCDWriter writer;
		  seg.setOptimizeCoefficients (true);
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (100);
		  seg.setDistanceThreshold (0.01);

		  int i=0, nr_points = (int) msg3->points.size ();
		  while (msg3->points.size () > 0.3 * nr_points)
		  {
		    // Segment the largest planar component from the remaining cloud
		    seg.setInputCloud (msg3);
		    seg.segment (*inliers, *coefficients);
		    if (inliers->indices.size () == 0)
		    {
		      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		      break;
		    }

		    // Extract the planar inliers from the input cloud
		    pcl::ExtractIndices<pcl::PointXYZ> extract;
		    extract.setInputCloud (msg3);
		    extract.setIndices (inliers);
		    extract.setNegative (false);

		    // Get the points associated with the planar surface
		    extract.filter (*cloud_plane);
		/*    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << "and total points were"<< msg3->points.size()<<std::endl;*/

		    // Remove the planar inliers, extract the rest
		    extract.setNegative (true);
		    extract.filter (*cloud_f);
		    *msg3 = *cloud_f;
		  }

	///////////////////////////////////////////////
 		pub_n2.publish(msg3);
                msg2->header.frame_id = "velodyne";
                msg2->height = msg->height;
                msg2->width = msg->width;
                BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
 		msg2->points.push_back (pcl::PointXYZ(pt.x,pt.y,0.0));
		//pub_n1.publish(msg2);

	// Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (msg2);

		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (0.3); // 2cm
		  ec.setMinClusterSize (20);
		  ec.setMaxClusterSize (2500);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (msg2);
		  ec.extract (cluster_indices);

		  int j = 0;
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != 				cluster_indices.end (); ++it)
		  {

		    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		      cloud_cluster->points.push_back (msg->points[*pit]); //*
		    cloud_cluster->width = cloud_cluster->points.size ();
		    cloud_cluster->height = 1;
		    cloud_cluster->is_dense = true;

		//    std::cout << "number of clusters: "<< j << std::endl;
		  j++;
		  }
		    std::cout << "number of clusters: "<< j << std::endl;
		cloud_cluster->header.frame_id = "velodyne";
		pub_n1.publish(cloud_cluster);
		};
         	};
project::project(){

     	        sub_n= n.subscribe("/velodyne_points", 1, &project::callback, this);
	        pub_n1 = n.advertise<PointCloud>("/vel2",1);
	        pub_n2 = n.advertise<PointCloud>("/vel3",1);
			}


int main(int argc,char** argv)
{
	ros::init(argc, argv, "project_pcl");
        project flatten;


ros::spin();

}
