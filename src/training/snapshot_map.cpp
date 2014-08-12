/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 13.06.2013
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
//#include <rosmrsmap/ObjectData.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include <rosmrsmap/SelectBoundingBox.h>
//#include <rosmrsmap/TriggerInitialAlignment.h>

#include <Eigen/Core>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <mrsmap/map/multiresolution_surfel_map.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>


#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"


#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"


#include <rosmrsmap/StringService.h>
#include <std_msgs/Int32.h>


using namespace mrsmap;



class SnapshotMap
{
public:

    SnapshotMap( ros::NodeHandle& nh ) : nh_( nh ) {

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );
		treeNodeAllocator_ = boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > >( new spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue >( 10000 ) );

		snapshot_service_ = nh.advertiseService("snapshot", &SnapshotMap::snapshotRequest, this);
		pub_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "output_cloud", 1);

		pub_status_ = nh.advertise< std_msgs::Int32 >( "status", 1 );

		tf_listener_ = boost::shared_ptr< tf::TransformListener >( new tf::TransformListener() );

		nh.param<double>( "max_resolution", max_resolution_, 0.0125 );
		nh.param<double>( "max_radius", max_radius_, 30.0 );

		nh.param<double>( "dist_dep_factor", dist_dep_, 0.005 );

		nh.param<std::string>( "map_folder", map_folder_, "." );

		nh.param<std::string>( "init_frame", init_frame_, "" );

		create_map_ = false;

		responseId_ = -1;

    }


	bool snapshotRequest( rosmrsmap::StringService::Request &req, rosmrsmap::StringService::Response &res ) {

		object_name_ = req.str;
		sub_cloud_ = nh_.subscribe( "input_cloud", 1, &SnapshotMap::dataCallback, this );
		create_map_ = true;

		ROS_INFO_STREAM( "subscribed at " << sub_cloud_.getTopic() );

		res.responseId = responseId_ + 1;

		return true;

	}


	void dataCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

		if( !create_map_ )
			return;

		ROS_INFO("creating map");


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::fromROSMsg(*point_cloud, *pointCloudIn);

//		Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();

		// change reference frame of point cloud to point mean and oriented along principal axes
		Eigen::Vector4d mean;
		Eigen::Vector3d eigenvalues;
		Eigen::Matrix3d cov;
		Eigen::Matrix3d eigenvectors;
		pcl::computeMeanAndCovarianceMatrix( *pointCloudIn, cov, mean );
		pcl::eigen33( cov, eigenvectors, eigenvalues );

		if( Eigen::Vector3d(eigenvectors.col(0)).dot( Eigen::Vector3d::UnitZ() ) > 0.0 )
			eigenvectors.col(0) = (-eigenvectors.col(0)).eval();

		// transform from object reference frame to camera
		Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();
		objectTransform.block<3,1>(0,0) = eigenvectors.col(2);
		objectTransform.block<3,1>(0,1) = eigenvectors.col(1);
		objectTransform.block<3,1>(0,2) = eigenvectors.col(0);
		objectTransform.block<3,1>(0,3) = mean.block<3,1>(0,0);

		if( objectTransform.block<3,3>(0,0).determinant() < 0 ) {
			objectTransform.block<3,1>(0,0) = -objectTransform.block<3,1>(0,0);
		}

		Eigen::Matrix4d objectTransformInv = objectTransform.inverse();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::transformPointCloud( *pointCloudIn, *objectPointCloud, (objectTransformInv).cast<float>() );

		objectPointCloud->sensor_origin_ = objectTransformInv.block<4,1>(0,3).cast<float>();
		objectPointCloud->sensor_orientation_ = Eigen::Quaternionf( objectTransformInv.block<3,3>(0,0).cast<float>() );

		treeNodeAllocator_->reset();
		map_ = boost::shared_ptr< MultiResolutionSurfelMap >( new MultiResolutionSurfelMap( max_resolution_, max_radius_, treeNodeAllocator_ ) );

		map_->params_.dist_dependency = dist_dep_;

		std::vector< int > pointIndices( objectPointCloud->points.size() );
		for( unsigned int i = 0; i < pointIndices.size(); i++ ) pointIndices[i] = i;
		map_->imageAllocator_ = imageAllocator_;
		map_->addPoints( *objectPointCloud, pointIndices );
		map_->octree_->root_->establishNeighbors();
		map_->evaluateSurfels();
		map_->buildShapeTextureFeatures();

		map_->save( map_folder_ + "/" + object_name_ + ".map" );

		if( init_frame_ != "" ) {

			ROS_INFO_STREAM( "using init frame " << init_frame_ << std::endl );

			try {

				tf::StampedTransform tf;
				tf_listener_->lookupTransform( init_frame_, point_cloud->header.frame_id, point_cloud->header.stamp, tf );

								Eigen::Affine3d init_frame_transform;
								tf::transformTFToEigen( tf, init_frame_transform );

				objectTransform = (init_frame_transform.matrix() * objectTransform).eval();


			}
				catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}


		}

		{

			Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );

			std::ofstream initPoseFile( map_folder_ + "/" + object_name_ + ".pose" );
			initPoseFile << "# x y z qx qy qz qw" << std::endl;
			initPoseFile << objectTransform(0,3) << " " << objectTransform(1,3) << " " << objectTransform(2,3) << " "
					 << q.x() << " " << q.y() << " " << q.z() << " "  << q.w() << std::endl;
			initPoseFile << "# init_pose: { position: { x: " << objectTransform(0,3) << ", y: " << objectTransform(1,3) << ", z: " << objectTransform(2,3)
					<< " }, orientation: { x: " << q.x() << ", y: " << q.y() << ", z: " << q.z() << ", w: " << q.w() << " } } }" << std::endl;
		}

		cloudv = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv->header.frame_id = object_name_;
		map_->visualize3DColorDistribution( cloudv, -1, -1, false );


		Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );

		object_tf_.setIdentity();
		object_tf_.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
		object_tf_.setOrigin( tf::Vector3( objectTransform(0,3), objectTransform(1,3), objectTransform(2,3) ) );

		object_tf_.stamp_ = point_cloud->header.stamp;
		object_tf_.child_frame_id_ = object_name_;

		if( init_frame_ == "" ) {
			object_tf_.frame_id_ = point_cloud->header.frame_id;
		}
		else
			object_tf_.frame_id_ = init_frame_;

		tf_broadcaster.sendTransform( object_tf_ );

		sub_cloud_.shutdown();

		create_map_ = false;

		responseId_++;

	}

	void update() {

		std_msgs::Int32 status;
		status.data = responseId_;
		pub_status_.publish( status );

		if( cloudv ) {

			object_tf_.stamp_ = ros::Time::now();
			tf_broadcaster.sendTransform( object_tf_ );

			std_msgs::Header header;
			header.frame_id = object_name_;
			header.stamp = object_tf_.stamp_;
			cloudv->header = pcl_conversions::toPCL( header );
			pub_cloud.publish( cloudv );
		}


	}


public:

	ros::NodeHandle nh_;
	ros::Subscriber sub_cloud_;
	ros::Publisher pub_status_;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_cloud;
	boost::shared_ptr< tf::TransformListener > tf_listener_;
	tf::TransformBroadcaster tf_broadcaster;

	double max_resolution_, max_radius_, dist_dep_;

	bool create_map_;

	ros::ServiceServer snapshot_service_;

	std::string map_folder_;
	std::string object_name_;
	std::string init_frame_;

	boost::shared_ptr< MultiResolutionSurfelMap > map_;
	tf::StampedTransform object_tf_;

	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv;

	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;
	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_;

	int responseId_;

};



int main(int argc, char** argv) {

	ros::init(argc, argv, "snapshot_map");
	ros::NodeHandle n("snapshot_map");
	SnapshotMap sm( n );

	ros::Rate r( 100 );
	while( ros::ok() ) {

		sm.update();

		ros::spinOnce();
		r.sleep();

	}


	return 0;
}

