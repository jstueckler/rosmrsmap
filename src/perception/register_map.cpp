/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 03.07.2014
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
#include <eigen_conversions/eigen_msg.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <mrsmap/map/multiresolution_surfel_map.h>

#include <mrsmap/registration/multiresolution_soft_surfel_registration.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>

#include <mrsmap/utilities/utilities.h>

#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/time.h"


#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"


#include <rosmrsmap/RegisterMap.h>
#include <rosmrsmap/ObjectTrackingData2.h>
#include <rosmrsmap/RegisterMapStatus.h>
#include <std_msgs/Int32.h>


using namespace mrsmap;



class RegisterMap
{
public:

    RegisterMap( ros::NodeHandle& nh ) : nh_( nh ) {

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );
		treeNodeAllocator_ = boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > >( new spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue >( 10000 ) );

		register_service_ = nh.advertiseService("register", &RegisterMap::registerRequest, this);
		pub_model_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "model_cloud", 1);
		pub_scene_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "scene_cloud", 1);

		pub_status_ = nh.advertise< rosmrsmap::RegisterMapStatus >( "status", 1 );
		
		pub_result_ = nh.advertise< rosmrsmap::ObjectTrackingData2 >( "result", 1 );

		tf_listener_ = boost::shared_ptr< tf::TransformListener >( new tf::TransformListener() );

		nh.param<std::string>( "map_folder", map_folder_, "." );

		nh.param<double>( "dist_dep_factor", dist_dep_, 0.005 );

		register_map_ = false;

		first_frame_ = false;

		responseId_ = 0;

		has_transform_ = false;

    }

    inline bool lookupTransform(const std::string &target_frame,
                              const std::string &source_frame,
                              Eigen::Matrix4f &transform,
                              const ros::Time& stamp = ros::Time(0))
    {
       tf::StampedTransform ros_transform;
       try
       {
         //        tf_listener_->lookupTransform (target_frame, cloud_in.header.frame_id, ros::Time(0), ros_transform);
         //        ROS_ERROR("%f", (ros::Time::now()-cloud_in.header.stamp).toSec());
         tf_listener_->waitForTransform( target_frame, source_frame, stamp, ros::Duration(0.1));
         tf_listener_->lookupTransform (target_frame, source_frame, stamp, ros_transform);
       }
       catch (tf::LookupException &e)
       {
         ROS_ERROR ("%s", e.what ());
         return (false);
       }
       catch (tf::ExtrapolationException &e)
       {
         ROS_ERROR ("%s", e.what ());
         try
         {
           tf_listener_->lookupTransform (target_frame, source_frame, ros::Time(0), ros_transform);
         }
         catch (tf::LookupException &e)
         {
           ROS_ERROR ("%s", e.what ());
           return (false);
         }
         catch (...)
         {
           ROS_ERROR ("UNKNOWN EXCEPTION");
           return false;
         }
         //        ROS_ERROR ("%s", e.what ());
         //        return (false);
       }
       catch (const tf::ConnectivityException& e)
       {
         ROS_ERROR ("%s", e.what ());
         return (false);
       }
       catch ( const std::exception& e )
       {
         ROS_ERROR ("%s", e.what ());
         return (false);
       }
       catch (...)
       {
         ROS_ERROR ("UNKNOWN EXCEPTION");
         return (false);
       }

       pcl_ros::transformAsMatrix(ros_transform, transform);

       return (true);
    }


	bool registerRequest( rosmrsmap::RegisterMap::Request &req, rosmrsmap::RegisterMap::Response &res ) {

		object_name_ = req.object_name;
		tf::poseMsgToEigen( req.init_pose, initial_pose_ );

		lastObjectTransform_.setIdentity();
		has_transform_ = false;

		init_frame_ = req.init_frame;

		if( object_name_ == "" ) {
			sub_cloud_.shutdown();
			register_map_ = false;
		}
		else {

			ROS_INFO("loading map");

			map_ = boost::shared_ptr< MultiResolutionSurfelMap >( new MultiResolutionSurfelMap( 0.0125f, 30.f ) );
			map_->load( map_folder_ + "/" + object_name_ + ".map" );
			map_->octree_->root_->establishNeighbors();
			map_->evaluateSurfels();
			map_->buildShapeTextureFeatures();

			map_->extents( model_mean_, model_cov_ );

			sub_cloud_ = nh_.subscribe( "input_cloud", 1, &RegisterMap::dataCallback, this );
			register_map_ = true;

			first_frame_ = true;

			track_ = req.track;
			once_ = req.once;

			ROS_INFO_STREAM( "subscribed at " << sub_cloud_.getTopic() );

		}

		responseId_++;
		res.responseId = responseId_;

		return true;

	}


	void update() {
		
		if( !register_map_ || !has_transform_ )
			return;

		if( once_ && !first_frame_ ) {
			
			// just send out result/status
			
			tf::StampedTransform object_tf;

			Eigen::Matrix4d objectTransform = lastObjectTransform_;

			object_tf.stamp_ = ros::Time::now();
			object_tf.child_frame_id_ = object_name_;
			object_tf.frame_id_ = output_frame_;

			Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );
			object_tf.setIdentity();
			object_tf.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
			object_tf.setOrigin( tf::Vector3( objectTransform(0,3), objectTransform(1,3), objectTransform(2,3) ) );

			tf_broadcaster.sendTransform( object_tf );
			

			rosmrsmap::RegisterMapStatus status;
			status.header.stamp = object_tf.stamp_;
			status.header.frame_id = object_tf.frame_id_;
			status.response_id = responseId_;
			status.confidence = lastConfidence_;
			pub_status_.publish( status );


			rosmrsmap::ObjectTrackingData2 msg;
			msg.header.stamp = object_tf.stamp_;
			msg.header.frame_id = object_tf.frame_id_;
			msg.object_name = object_name_;
			tf::poseEigenToMsg( Eigen::Affine3d( objectTransform ), msg.object_pose );
			msg.requestID = responseId_;
			pub_result_.publish( msg );
			
			return;
			
		}

	}


	void dataCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

		if( !register_map_ )
			return;

		if( once_ && !first_frame_ ) {
			return;
		}

		ROS_INFO("registering");
		pcl::StopWatch sw;


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::fromROSMsg(*point_cloud, *pointCloudIn);


		// create mrsmap from pointcloud
		treeNodeAllocator_->reset();
		boost::shared_ptr< MultiResolutionSurfelMap > currMap = boost::shared_ptr< MultiResolutionSurfelMap >( new MultiResolutionSurfelMap( map_->min_resolution_, map_->max_range_, treeNodeAllocator_ ) );
		currMap->imageAllocator_ = imageAllocator_;

		currMap->params_.dist_dependency = dist_dep_;


		std::vector< int > imageBorderIndices;

//		std::vector< int > pointIndices( pointCloudIn->points.size() );
//		for( unsigned int i = 0; i < pointIndices.size(); i++ ) pointIndices[i] = i;
//		currMap->addPoints( *pointCloudIn, pointIndices );

		cv::Mat img_rgb( 480, 640, CV_8UC3, 0.f );
		cv::Mat img_depth( 480, 640, CV_16UC1, 0.f );
		mrsmap::reprojectPointCloudToImages( pointCloudIn, img_rgb, img_depth );
		mrsmap::imagesToPointCloud( img_depth, img_rgb, "0", pointCloudIn );
		currMap->addImage( *pointCloudIn, false, true );

		currMap->octree_->root_->establishNeighbors();
		currMap->markNoUpdateAtPoints( *pointCloudIn, imageBorderIndices );
		currMap->evaluateSurfels();
		currMap->buildShapeTextureFeatures();

		Eigen::Affine3d initPose = initial_pose_;
		if( !track_ || (track_ && first_frame_) ) {

			// transform init pose to camera frame
			if( init_frame_ != "" ) {

				ROS_INFO_STREAM( "using init frame " << init_frame_ << std::endl );

				Eigen::Matrix4f init_frame_transform;
				if( lookupTransform( init_frame_, point_cloud->header.frame_id, init_frame_transform, point_cloud->header.stamp ) ) {
					initPose = Eigen::Affine3d(init_frame_transform.cast<double>().inverse() * initial_pose_.matrix());
				}
			}

		}


		// register scene to model
//		Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d transform = initPose.matrix().inverse();

		// initialize alignment by shifting the map centroids
		Eigen::Vector3d scene_mean;
		Eigen::Matrix3d scene_cov;
		currMap->extents( scene_mean, scene_cov );

		Eigen::Vector4d scene_mean4;
		scene_mean4.head<3>() = scene_mean;
		scene_mean4(3) = 1.0;
		Eigen::Vector4d Tscene_mean4 = transform * scene_mean4;

		if( (track_ && first_frame_) || !track_ ) {
			transform.block<3,1>(0,3) += model_mean_ - Tscene_mean4.head<3>();
			first_frame_ = false;
		}

		pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrSrc;
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrTgt;

//#if SOFT_REGISTRATION
		MultiResolutionSoftSurfelRegistration reg;
		reg.params_.match_likelihood_use_color_ = false;
		reg.estimateTransformation( *map_, *currMap, transform, 32.f * currMap->min_resolution_, currMap->min_resolution_, corrSrc, corrTgt, 100 );
//#else
//		MultiResolutionSurfelRegistration reg;
//		reg.estimateTransformation( *map_, *currMap, transform, 32.f * currMap->min_resolution_, currMap->min_resolution_, corrSrc, corrTgt, 100, 0, 5 );
//#endif

		ROS_INFO_STREAM( "registering took " << sw.getTimeSeconds() );

		// visualize model
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv->header = pointCloudIn->header;
		map_->visualize3DColorDistribution( cloudv, -1, -1, false );
		pcl::transformPointCloud( *cloudv, *cloudv, transform.inverse().cast<float>() );
		pub_model_cloud.publish( cloudv );

		// visualize scene
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv2->header = pointCloudIn->header;
		currMap->visualize3DColorDistribution( cloudv2, -1, -1, false );
//		pcl::transformPointCloud( *cloudv2, *cloudv2, transform );
		pub_scene_cloud.publish( cloudv2 );


		tf::StampedTransform object_tf;

		Eigen::Matrix4d objectTransform = transform.inverse();

		Eigen::Matrix4f cameraToInitTransform = Eigen::Matrix4f::Identity();
		if( lookupTransform( init_frame_, point_cloud->header.frame_id, cameraToInitTransform, point_cloud->header.stamp ) ) {

			objectTransform =  (cameraToInitTransform.cast<double>() * objectTransform).eval();

			object_tf.stamp_ = point_cloud->header.stamp;
			object_tf.child_frame_id_ = object_name_;
			object_tf.frame_id_ = init_frame_;

			output_frame_ = init_frame_;

		}
		else {

			object_tf.stamp_ = point_cloud->header.stamp;
			object_tf.child_frame_id_ = object_name_;
			object_tf.frame_id_ = point_cloud->header.frame_id;

			output_frame_ = point_cloud->header.frame_id;

		}

		Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );
		object_tf.setIdentity();
		object_tf.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
		object_tf.setOrigin( tf::Vector3( objectTransform(0,3), objectTransform(1,3), objectTransform(2,3) ) );

		tf_broadcaster.sendTransform( object_tf );

		lastObjectTransform_ = objectTransform;

		has_transform_ = true;



		double confidence = 1.0;

		// determine confidence using matching likelihood
		MultiResolutionSurfelRegistration reg_hard;
		reg_hard.params_.match_likelihood_use_color_ = false;
		reg_hard.params_.model_visibility_max_depth_ = 13;

		double matchLogLikelihood = reg_hard.matchLogLikelihood( *map_, *currMap, transform );
		double selfLogLikelihood = reg_hard.selfMatchLogLikelihood( *map_ );

		confidence = std::min( 1.0, std::max( 0.0, 4.0 * (matchLogLikelihood / selfLogLikelihood - 0.5) ) );

//		ROS_INFO_STREAM( "detection match log likelihood: " << matchLogLikelihood );
//		ROS_INFO_STREAM( "detection self log likelihood: " << selfLogLikelihood );
//		ROS_INFO_STREAM( "detection confidence: " << confidence );


		lastConfidence_ = confidence;





		rosmrsmap::RegisterMapStatus status;
		status.header.stamp = object_tf.stamp_;
		status.header.frame_id = object_tf.frame_id_;
		status.response_id = responseId_;
		status.confidence = lastConfidence_;
		pub_status_.publish( status );

		rosmrsmap::ObjectTrackingData2 msg;
		msg.header.stamp = object_tf.stamp_;
		msg.header.frame_id = object_tf.frame_id_;
		msg.object_name = object_name_;
		tf::poseEigenToMsg( Eigen::Affine3d( objectTransform ), msg.object_pose );
		msg.requestID = responseId_;
		pub_result_.publish( msg );


		if( track_ )
			initial_pose_ = Eigen::Affine3d( transform.inverse().eval() );

	}




public:

	ros::NodeHandle nh_;
	ros::Subscriber sub_cloud_;
	ros::Publisher pub_status_, pub_result_;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_model_cloud, pub_scene_cloud;
	boost::shared_ptr< tf::TransformListener > tf_listener_;
	tf::TransformBroadcaster tf_broadcaster;

	bool register_map_;
	bool track_;
	bool first_frame_;
	bool once_;
	bool has_transform_;

	std::string init_frame_, output_frame_;

	boost::shared_ptr< MultiResolutionSurfelMap > map_;
	Eigen::Vector3d model_mean_;
	Eigen::Matrix3d model_cov_;

	ros::ServiceServer register_service_;

	std::string map_folder_;
	std::string object_name_;
	Eigen::Affine3d initial_pose_;

	Eigen::Matrix4d lastObjectTransform_;
	double lastConfidence_;

	double dist_dep_;

	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;
	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_;

	int responseId_;

};



int main(int argc, char** argv) {

	ros::init(argc, argv, "register_map");
	ros::NodeHandle n("register_map");
	RegisterMap sm( n );

	ros::Rate r( 30 );
	while( ros::ok() ) {

		sm.update();

		ros::spinOnce();
		r.sleep();

	}


	return 0;
}

