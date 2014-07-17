/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 27.06.2011
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

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>

#include <pcl/registration/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosmrsmap/ObjectTrackingRequest.h>
#include <rosmrsmap/ObjectTrackingData2.h>

#include <mrsmap/map/multiresolution_surfel_map.h>

#define SOFT_REGISTRATION 0
#if SOFT_REGISTRATION
#include <mrsmap/registration/multiresolution_soft_surfel_registration.h>
#else
#include <mrsmap/registration/multiresolution_surfel_registration.h>
#endif

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


using namespace mrsmap;


/**
 * track object on service request.
 * request contains initial pose guess, pose in base_link coordinates (translation == position of map_mean, z-dir should be upwards, so you can turn the object by yaw rotation.
 */
class ObjectTrackingNode {
public:

	ObjectTrackingNode() {

	}

	~ObjectTrackingNode() {
	}

	void onInit(ros::NodeHandle& nh) {

		// parameters
		//    	nh.param< std::string >( "bag_name", bag_name_, "trainingdata.bag" );

		// publishers / subscribers
		sub_cloud = nh.subscribe( "input_cloud", 1, &ObjectTrackingNode::dataCallback, this );
		pub_viz = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array", 10);
		pub_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "output_cloud", 1);
		pub_model_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "output_model_cloud", 1);
		pub_data = nh.advertise<rosmrsmap::ObjectTrackingData2> ("output_tracking_data", 1);
		pub_pose = nh.advertise<geometry_msgs::Pose>("object_pose", 1);

		// services
		objectTrackingServer_ = nh.advertiseService( "track_objects", &ObjectTrackingNode::objectTrackingRequestReceived, this );

		map_ = new MultiResolutionSurfelMap( 0.1f, 30.f ); // init values dont mean anything
		requestId = 0;

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );
		treeNodeAllocator_ = boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > >( new spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue >( 10000 ) );

	}


	void update() {

	}

	void dataCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

//		map_->header = point_cloud->header;

		if(req_.enable == 0)
			return;

		pcl::PointCloud<pcl::PointXYZRGB> pointCloudIn;

		try {
//			ROS_INFO("[%s]: converting point cloud...", __PRETTY_FUNCTION__);
			pcl::fromROSMsg(*point_cloud, pointCloudIn);
		} catch (std::exception &e) {
			ROS_ERROR("Exception while converting point cloud");
			return;
		}

		const int numPoints = pointCloudIn.points.size();

		std::vector<int> indices(numPoints);
		for (int i = 0; i < numPoints; i++)
			indices[i] = i;

		const float min_resolution = map_->min_resolution_;
		const float max_radius = map_->max_range_;


	//	Eigen::Matrix< double, 6, 1 > max_deviations;
	//	max_deviations(0) = req_.pose_guess_max_x_dev;
	//	max_deviations(1) = req_.pose_guess_max_y_dev;
	//	max_deviations(2) = req_.pose_guess_max_z_dev;
	//	max_deviations(3) = req_.pose_guess_max_roll_dev;
	//	max_deviations(4) = req_.pose_guess_max_pitch_dev;
	//	max_deviations(5) = req_.pose_guess_max_yaw_dev;



		double logLikelihood = 0;
		if( true )
		{
			// incremental registration..
			Eigen::Matrix4f transform = lastTransform;

			// transform point cloud..
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::transformPointCloud(pointCloudIn, *transformedCloud, transform);

			transformedCloud->sensor_origin_ = transform.block<4, 1> (0, 3);
			transformedCloud->sensor_orientation_ = Eigen::Quaternionf( transform.block<3, 3> (0, 0));

			Eigen::Matrix4d incTransform;
			incTransform.setIdentity();


			// consider image borders
			std::vector< int > imageBorderIndices;
			map_->findVirtualBorderPoints( *transformedCloud, imageBorderIndices );

			Eigen::Vector3d t_est( transform(0,3), transform(1,3), transform(2,3) );
			double dist_est = (t_est - map_mean_).norm();

			double covfactor = std::max( 1.5, 0.5*dist_est );

			// concentrate on window at last object pose estimate
			for( unsigned int i = 0; i < transformedCloud->points.size(); i++ ) {

				const pcl::PointXYZRGB& p = transformedCloud->points[i];
				if( isnan( p.x ) ) {
					continue;
				}

				Eigen::Vector3d point( p.x, p.y, p.z );
				if( (point-map_mean_).dot( map_cov_inv_ * (point-map_mean_) ) > covfactor*8.0 ) {
					transformedCloud->points[i].x = std::numeric_limits< float >::quiet_NaN();
				}

			}


			transformedCloud->header.frame_id = "base_link";
			transformedCloud->header.stamp = pointCloudIn.header.stamp;

			indices.clear();
			indices.resize( transformedCloud->points.size() );
			for( unsigned int ind = 0; ind < indices.size(); ind++ )
				indices[ind] = ind;

			transformedCloud->sensor_origin_ = transform.block<4,1>(0,3);
			transformedCloud->sensor_orientation_ = Eigen::Quaternionf( transform.block<3,3>(0,0) );


			pcl::StopWatch sw;
			sw.reset();

			// add points to local map
			treeNodeAllocator_->reset();
			MultiResolutionSurfelMap target( map_->min_resolution_, map_->max_range_, treeNodeAllocator_ );
			target.imageAllocator_ = imageAllocator_;
//			target.addImage( *transformedCloud, false );
			target.addPoints( *transformedCloud, indices );
			target.octree_->root_->establishNeighbors();
//			target.markNoUpdateAtPoints( *transformedCloud, imageBorderIndices );
			target.evaluateSurfels();
			target.buildShapeTextureFeatures();


			pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrSrc;
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrTgt;

#if SOFT_REGISTRATION
		MultiResolutionSoftSurfelRegistration reg;
		
		Eigen::Matrix< double, 6, 1 > prior_pose_mean = Eigen::Matrix< double, 6, 1 >::Zero();
		Eigen::Matrix< double, 6, 1 > prior_pose_variances = 1.0 * Eigen::Matrix< double, 6, 1 >::Ones();
		prior_pose_variances(3) = prior_pose_variances(4) = 0.0001;
		reg.setPriorPose( true, prior_pose_mean, prior_pose_variances );
		
		bool retVal = reg.estimateTransformation( *map_, target, incTransform, req_.register_start_resolution, req_.register_stop_resolution, corrSrc, corrTgt, 100 );
#else
		MultiResolutionSurfelRegistration reg;
		
		Eigen::Matrix< double, 6, 1 > prior_pose_mean = Eigen::Matrix< double, 6, 1 >::Zero();
		Eigen::Matrix< double, 6, 1 > prior_pose_variances = 1.0 * Eigen::Matrix< double, 6, 1 >::Ones();
		prior_pose_variances(3) = prior_pose_variances(4) = 0.0001;
		reg.setPriorPose( true, prior_pose_mean, prior_pose_variances );
		
		bool retVal = reg.estimateTransformation( *map_, target, incTransform, req_.register_start_resolution, req_.register_stop_resolution, corrSrc, corrTgt, 100, 0, 5 );
#endif


			if( !retVal )
				std::cout << "registration failed!\n";


			double deltat = sw.getTime();

			ROS_INFO_STREAM( "registration took: " << deltat );

			if( retVal && !isnan(incTransform(0,0)) ) {

				Eigen::Matrix4f incTransformf = incTransform.cast<float>();

				transform = incTransformf * lastTransform;

		//		Eigen::Matrix4d transformd = Eigen::Matrix4d(transform.cast<double> ());
		//		logLikelihood = registration::matchLogLikelihood(*map_, target, transformd);
		//		ROS_INFO("log likelihood: %lf", logLikelihood);

				lastTransform = transform;

				pub_cloud.publish(transformedCloud);
			}

		}


		// publish transform as tf with frame_id req_.object_name
		// obj pose in base_link = T_obj_to_baselink = T_camera_to_baselink * T_map_to_camera * T_obj_to_map

		tf::Transform transform;
		transform.setIdentity();
		transform.setOrigin( tf::Vector3( lastTransform(0,3), lastTransform(1,3), lastTransform(2,3) ) );
		Eigen::Quaternionf q( Eigen::Matrix3f( lastTransform.block<3,3>(0,0) ) );
		transform.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
		// transform: camera to map transform
		transform = transform.inverse();
		// now transforms from map to camera

		tf::StampedTransform baseTransform;
		baseTransform.setIdentity();
		try {
			// target, source, time
			tf_listener_.lookupTransform( "base_link", point_cloud->header.frame_id, ros::Time(0), baseTransform );
		}
		catch(tf::TransformException& ex) {
			ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", point_cloud->header.frame_id.c_str(), "base_link", ex.what());
		}

//		ROS_ERROR("camera to base_link %f %f %f", baseTransform.getOrigin().x(), baseTransform.getOrigin().y(), baseTransform.getOrigin().z());

		tf::Transform objInMapTransform;
		objInMapTransform.setIdentity();

		if( req_.center_map )
			objInMapTransform.setOrigin( tf::Vector3( map_mean_(0), map_mean_(1), map_mean_(2) ) );

		tf::Transform objPose = baseTransform * transform * objInMapTransform;


		// publish data, this is the right transform direction for m_targetPose in mobmanip fsm
		rosmrsmap::ObjectTrackingData2 trackingData;
		trackingData.header.frame_id = "base_link";
		trackingData.header.stamp = point_cloud->header.stamp;
		trackingData.object_name = req_.object_name;
		trackingData.object_pose.position.x = objPose.getOrigin().x();
		trackingData.object_pose.position.y = objPose.getOrigin().y();
		trackingData.object_pose.position.z = objPose.getOrigin().z();
		trackingData.object_pose.orientation.x = objPose.getRotation().x();
		trackingData.object_pose.orientation.y = objPose.getRotation().y();
		trackingData.object_pose.orientation.z = objPose.getRotation().z();
		trackingData.object_pose.orientation.w = objPose.getRotation().w();
		trackingData.log_likelihood = logLikelihood;
		trackingData.requestID = requestId;

		pub_data.publish( trackingData );

	//	tf_broadcaster_.sendTransform( tf::StampedTransform( transform, point_cloud->header.stamp, point_cloud->header.frame_id, req_.object_name ) );
		tf_broadcaster_.sendTransform( tf::StampedTransform( objPose, point_cloud->header.stamp, "base_link", req_.object_name ) );

	//		std::cout << "pos: " << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << "\n";
	//		std::cout << "rot: " << transform.getRotation().x() << " " << transform.getRotation().y() << " " << transform.getRotation().z() << " " << transform.getRotation().w() << "\n";

		if( pub_model_cloud.getNumSubscribers() > 0 ) {
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
			cloud->header = pointCloudIn.header;
//			cloud->header.stamp = ros::Time::now();
			map_->visualize3DColorDistribution( cloud, -1, -1, false );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
			Eigen::Matrix4f lTinv = lastTransform.inverse();
			pcl::transformPointCloud(*cloud, *transformedCloud, lTinv );
			pub_model_cloud.publish( transformedCloud );
		}

		geometry_msgs::Pose objPoseMsg;
		tf::poseTFToMsg( objPose, objPoseMsg );
		pub_pose.publish( objPoseMsg );


	//		Eigen::Matrix4d diffTransform = baseEigenTransform.inverse() * lastTransform.cast<double>().inverse() * baseEigenTransform;
	//		Eigen::Quaterniond qDiff( Eigen::Matrix3d( diffTransform.block<3,3>(0,0) ) );
	//		btMatrix3x3 btRotDiff( btQuaternion( qDiff.x(), qDiff.y(), qDiff.z(), qDiff.w() ) );
	//		double yaw = 0, pitch = 0, roll = 0;
	//		// TODO: check if this extracts what i want
	//		btRotDiff.getEulerYPR( yaw, pitch, roll );
	//		ROS_ERROR("%f %f %f %f %f %f", diffTransform(0,3), diffTransform(1,3), diffTransform(2,3), roll, pitch, yaw);

	}

	bool objectTrackingRequestReceived( rosmrsmap::ObjectTrackingRequest::Request &req, rosmrsmap::ObjectTrackingRequest::Response &res ) {
		
		req_ = req;

		if( req.enable == 0 )
			return true;

		map_->load( ros::package::getPath("nimbro_launch") + "/objectknowledge/" + req.object_name + ".map" );

		map_->setApplyUpdate( true );
		map_->octree_->root_->establishNeighbors();
//		map_->setUpToDate(false);
		map_->buildShapeTextureFeatures();

		map_->extents( map_mean_, map_cov_ );
		map_cov_inv_ = map_cov_.inverse();

		ROS_INFO_STREAM( map_mean_ );

		// convert geometry_msg/Pose to Eigen
		req_transform_guess_.setIdentity();
		Eigen::Quaternionf q( req.pose_guess.orientation.w, req.pose_guess.orientation.x, req.pose_guess.orientation.y, req.pose_guess.orientation.z );
		req_transform_guess_.block<3,3>(0,0) = Eigen::Matrix3f(q);
		req_transform_guess_(0,3) = req.pose_guess.position.x;
		req_transform_guess_(1,3) = req.pose_guess.position.y;
		req_transform_guess_(2,3) = req.pose_guess.position.z;


		// assumption: rotation of object frame corresponds to rotation of map reference frame
		// however, we obtain the object frame by translating the map ref. frame to the object center (map mean)
		// we report object pose then as the transform between object frame to base_link.
		// we also get the pose guess in this format, so we have to retrieve map ref.frame to camera frame from it.
		// pose guess = T_obj_to_baselink = T_camera_to_baselink * T_map_to_camera * T_obj_to_map
		// T_map_to_camera = T_camera_to_baselink^-1 * T_obj_to_baselink * T_obj_to_map^-1

		// get T_camera_to_baselink
		tf::StampedTransform baseTransform;
		baseTransform.setIdentity();
		try {
			// target, source, time
			// this gets camera to baselink
			tf_listener_.lookupTransform( "base_link", "openni_rgb_optical_frame", ros::Time(0), baseTransform );
		}
		catch(tf::TransformException& ex) {
			baseTransform.setIdentity();
			ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", "base_link", "openni_rgb_optical_frame", ex.what());
		}

		Eigen::Matrix4f baseEigenTransform;
		baseEigenTransform.setIdentity();
		baseEigenTransform.block<3,3>(0,0) = Eigen::Matrix3f( Eigen::Quaternionf( baseTransform.getRotation().w(), baseTransform.getRotation().x(), baseTransform.getRotation().y(), baseTransform.getRotation().z() ) );
		baseEigenTransform(0,3) = baseTransform.getOrigin().x();
		baseEigenTransform(1,3) = baseTransform.getOrigin().y();
		baseEigenTransform(2,3) = baseTransform.getOrigin().z();

		ROS_INFO_STREAM( baseEigenTransform );

		Eigen::Matrix4f objInMapTransform;
		objInMapTransform.setIdentity();

		if( req.center_map )
			objInMapTransform.block<3,1>(0,3) = map_mean_.cast<float>();

		// T_map_to_camera = T_camera_to_baselink^-1 * T_obj_to_baselink * T_obj_to_map^-1
		req_transform_guess_ = (baseEigenTransform.inverse() * req_transform_guess_ * objInMapTransform.inverse()).eval();
		req_transform_guess_ = (req_transform_guess_.inverse()).eval();
		lastTransform = req_transform_guess_;



		requestId++;
		res.responseId = requestId;

		return true;

	}

public:

	ros::Subscriber sub_cloud;

	ros::Publisher pub_viz, pub_data, pub_pose;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_cloud;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_model_cloud;

	tf::TransformBroadcaster tf_broadcaster_;
	tf::TransformListener tf_listener_;

	MultiResolutionSurfelMap* map_;
	Eigen::Vector3d map_mean_;
	Eigen::Matrix3d map_cov_, map_cov_inv_;


	Eigen::Matrix4f lastTransform, req_transform_guess_;

	ros::ServiceServer objectTrackingServer_;
	rosmrsmap::ObjectTrackingRequest::Request req_;

	ros::Time lastInitTime;

	unsigned int requestId;


	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;
	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_;
};





int main(int argc, char** argv) {

	ros::init(argc, argv, "object_tracking_node");
	ros::NodeHandle n("object_tracking_node");
	ObjectTrackingNode otn;
	otn.onInit(n);

	ros::Rate loop_rate(100);
	while (n.ok()) {
		ros::spinOnce();
		otn.update();
		loop_rate.sleep();
	}

}

