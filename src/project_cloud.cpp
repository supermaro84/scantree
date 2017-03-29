#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class project{
	private:
		ros::NodeHandle n;
		ros::Publisher pub_n;
		ros::Subscriber sub_n;
	public:
		project();
		void callback(const PointCloud::ConstPtr &msg)
		{

                //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		PointCloud::Ptr msg2 (new PointCloud);
  		msg2->header.frame_id = "velodyne2";
 		msg2->height = msg->height;
		msg2->width = msg->width;
                BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
 		msg2->points.push_back (pcl::PointXYZ(pt.x,pt.y,0.0));
		//PointCloud msg2=msg;
		pub_n.publish(msg2);
		};

};
project::project(){

     	        sub_n= n.subscribe("/velodyne_points", 1, &project::callback, this);
	        pub_n = n.advertise<PointCloud>("/vel2",1);
			}


int main(int argc,char** argv)
{
	ros::init(argc, argv, "project_pcl");
        project flatten;

ros::spin();

}
