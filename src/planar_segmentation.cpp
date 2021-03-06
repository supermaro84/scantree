#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("velodyne_points", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_segmented", 1);
	pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_segmented", 1);
        ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
        coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented2;

        pcl::fromROSMsg(input, cloud);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.2);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.segment(*inliers, coefficients);

        // Publish the model coefficients
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        coef_pub.publish(ros_coefficients);

        // Publish the Point Indices
        pcl_msgs::PointIndices ros_inliers;
        pcl_conversions::fromPCL(*inliers, ros_inliers);
        ind_pub.publish(ros_inliers);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::ExtractIndices<pcl::PointXYZ> extract2;
        extract.setInputCloud(cloud.makeShared());
        extract2.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract2.setIndices(inliers);
        extract.setNegative(true);
	extract2.setNegative(false);
        extract.filter(cloud_segmented);
	extract2.filter(cloud_segmented2);

        //Publish the new cloud
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg(cloud_segmented, output);
        pcl::toROSMsg(cloud_segmented2, output2);
        pcl_pub.publish(output);
        pcl2_pub.publish(output2);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub, pcl2_pub, ind_pub, coef_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_planar_segmentation");

    cloudHandler handler;

    ros::spin();

    return 0;
}

