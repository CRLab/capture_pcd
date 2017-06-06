#include <ros/ros.h>
#include <ros/package.h>

#include <shape_completion_msgs/CapturePartialPCDAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <actionlib/server/simple_action_server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>



namespace pcd_capture
{

    class PCDCapture
    {
        private:

            std::vector<sensor_msgs::PointCloud2> pc_msgs;

            ros::NodeHandle nh;
            boost::mutex mutex;

            int cloud_queue_size;

            tf::TransformListener *tf_listener;

            //parameter values for statistical outlier removal
            int sor_MeanK;
            float sor_StddevMulThresh;

            ros::Subscriber pointCloudSubscriber;

            actionlib::SimpleActionServer<shape_completion_msgs::CapturePartialPCDAction> as_;
            void executeCB(const shape_completion_msgs::CapturePartialPCDGoalConstPtr &goal);
            void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

        public:
            PCDCapture(tf::TransformListener *_listener);
    };


    PCDCapture::PCDCapture(tf::TransformListener *_listener):
        nh("mesh_builder_node"),
        as_(nh, "/get_partial_pc", boost::bind(&PCDCapture::executeCB, this, _1), false),
        cloud_queue_size(5),
        sor_MeanK(50),
        sor_StddevMulThresh(0.0003),
        tf_listener(_listener)
    {
        as_.start();

        pointCloudSubscriber =  nh.subscribe("/filtered_pc", 10,  &PCDCapture::pointCloudCB, this);

        ROS_INFO("capture_pcd node ready\n");
    }



    void PCDCapture::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        boost::lock_guard<boost::mutex> lock(mutex);
        sensor_msgs::PointCloud2 pc_msg = *msg;
        pc_msgs.push_back(pc_msg);
        if (pc_msgs.size() > cloud_queue_size)
        {
            pc_msgs.erase(pc_msgs.begin());
        }
    }

    void PCDCapture::executeCB(const shape_completion_msgs::CapturePartialPCDGoalConstPtr &goal)
    {
        boost::lock_guard<boost::mutex> lock(mutex);

        ROS_INFO("executing complete mesh callback\n");

        shape_completion_msgs::CapturePartialPCDResult result_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr noOutlierCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud (new pcl::PointCloud<pcl::PointXYZRGB>());


        for(int i=0; i < pc_msgs.size(); i++)
        {
            //First Get a pointcloud
            pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(pc_msgs.at(i), *pcl_pc);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::iterator p;
            for (p = cloud->points.begin(); p < cloud->points.end(); p++)
            {
                combinedCloud->push_back((*p));
            }
        }

        //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        //sor.setInputCloud (combinedCloud);
        //sor.setMeanK (sor_MeanK);
        //sor.setStddevMulThresh (sor_StddevMulThresh);
        //sor.filter (*noOutlierCloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> vgridfilter;
        vgridfilter.setInputCloud (combinedCloud);
        vgridfilter.setLeafSize (0.003f, 0.003f, 0.003f);
        vgridfilter.filter (*downsampledCloud);

        tf::StampedTransform transform;

        std::string frame_id = std::string("/camera_rgb_optical_frame");
        std::string output_frame_id = std::string("/world");
        ros::Time now = ros::Time::now();
//        nh.getParam("frame_id", "frame_id");
//        nh.getParam("output_frame_id", output_frame_id);
        std::cout << "frame_id: " << frame_id.c_str() << std::endl;
        std::cout << "output_frame_id: " << output_frame_id.c_str() << std::endl;




	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (downsampledCloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (downsampledCloud);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	 {

	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_in_object_frame (new pcl::PointCloud<pcl::PointXYZRGB>);

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		  cloud_cluster->points.push_back (downsampledCloud->points[*pit]);
		  }

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		ROS_DEBUG("Waiting for world Transform");
		tf_listener->waitForTransform (output_frame_id, frame_id, now, ros::Duration(2.0));
		ROS_DEBUG("Looking up Transform");
		tf_listener->lookupTransform (output_frame_id, frame_id, now, transform);

		cloud_cluster->header.frame_id = frame_id;
		pcl_ros::transformPointCloud(output_frame_id,
					     *cloud_cluster,
			*cloud_cluster_transformed,
			*tf_listener);


	    Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
	    pcl::compute3DCentroid (*cloud_cluster, centroid); centroid.w () = 1.f;

            std::cout << "centroid.x(): " << centroid.x() << std::endl;
            std::cout << "centroid.y(): " << centroid.y() << std::endl;
            std::cout << "centroid.z(): " << centroid.z() << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::iterator p;
            int num_pts = 0;
            for (p = cloud_cluster->points.begin(); p < cloud_cluster->points.end(); p++)
            {
                pcl::PointXYZRGB *point = new pcl::PointXYZRGB;

                point->x = p->x - centroid.x();
                point->y = p->y - centroid.y();
                point->z = p->z;
                point->r = p->r;
                point->g = p->g;
                point->b = p->b;
                point->a = p->a;

                partial_in_object_frame->points.push_back(*point);
                num_pts ++;
            }
            partial_in_object_frame->width = num_pts;
            partial_in_object_frame->height = 1;

            std::string j_str;
            std::ostringstream convert;
            convert << j;
            j_str = convert.str();

            std::string partial_cf_filepath = goal->result_dir + std::string("partial_cf_")  + j_str + std::string(".pcd") ;
            std::string partial_of_filepath = goal->result_dir + std::string("partial_of_") + j_str + std::string(".pcd") ;
            std::string camera2world_filepath = goal->result_dir + std::string("camera_to_world_") + j_str + std::string(".txt") ;
            std::string world2partial_filepath = goal->result_dir + std::string("world_to_partial_") + j_str + std::string(".txt") ;

            ROS_INFO_STREAM("SAVING camera2world to: " << camera2world_filepath.c_str() << std::endl);
            std::ofstream camera2world_fh;
            camera2world_fh.open (camera2world_filepath.c_str());
            camera2world_fh << "frame1, frame2, x, y, z, qw, qx, qy, qz" << std::endl;
            camera2world_fh << frame_id.c_str() << ", " << output_frame_id << ", " ;
            camera2world_fh	<< transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << ", ";
            camera2world_fh	<< transform.getRotation().w() << ", " << transform.getRotation().x() << ", " << transform.getRotation().y() << ", " << transform.getRotation().z() << std::endl;
            camera2world_fh.close();

            ROS_INFO_STREAM("SAVING world2partial to: " << camera2world_filepath.c_str() << std::endl);
            std::ofstream world2partial_fh;
            world2partial_fh.open (world2partial_filepath.c_str());
            world2partial_fh << "frame1, frame2, x, y, z, qw, qx, qy, qz" << std::endl;
            world2partial_fh << "world" << ", " << "partial" << ", " ;
            world2partial_fh << centroid.x() << ", " << centroid.y() << ", 0, ";
            world2partial_fh << 1 << ", 0, 0, 0" << std::endl;
            world2partial_fh.close();

            ROS_INFO_STREAM("SAVING partial_cf to: " << partial_cf_filepath.c_str() << std::endl);
            pcl::io::savePCDFileBinary( partial_cf_filepath, *cloud_cluster_transformed);

            ROS_INFO_STREAM("SAVING partial_of to: " << partial_of_filepath.c_str() << std::endl);
            pcl::io::savePCDFileASCII( partial_of_filepath, *partial_in_object_frame);

            j++;

         }


        as_.setSucceeded(result_);
    }


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mesh_builder_node");
  ros::NodeHandle nh;

  tf::TransformListener tfl;
  pcd_capture::PCDCapture node(&tfl);

  ros::spin();

  return 0;
}
