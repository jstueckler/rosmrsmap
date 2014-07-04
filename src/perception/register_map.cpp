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

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <mrsmap/map/multiresolution_surfel_map.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>
#include <mrsmap/registration/multiresolution_soft_surfel_registration.h>


#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/time.h"


#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"


#include <rosmrsmap/StringService.h>
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

		pub_status_ = nh.advertise< std_msgs::Int32 >( "status", 1 );

		tf_listener_ = boost::shared_ptr< tf::TransformListener >( new tf::TransformListener() );

		nh.param<std::string>( "map_folder", map_folder_, "." );


		register_map_ = false;

		responseId_ = -1;

    }


	bool registerRequest( rosmrsmap::StringService::Request &req, rosmrsmap::StringService::Response &res ) {

		object_name_ = req.str;

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

			ROS_INFO_STREAM( "subscribed at " << sub_cloud_.getTopic() );

		}

		responseId_++;
		res.responseId = responseId_;

		return true;

	}


	void dataCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

		if( !register_map_ )
			return;

		ROS_INFO("registering");
		pcl::StopWatch sw;


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::fromROSMsg(*point_cloud, *pointCloudIn);


		// create mrsmap from pointcloud
		treeNodeAllocator_->reset();
		boost::shared_ptr< MultiResolutionSurfelMap > currMap = boost::shared_ptr< MultiResolutionSurfelMap >( new MultiResolutionSurfelMap( map_->min_resolution_, map_->max_range_, treeNodeAllocator_ ) );

		std::vector< int > pointIndices( pointCloudIn->points.size() );
		for( unsigned int i = 0; i < pointIndices.size(); i++ ) pointIndices[i] = i;

		std::vector< int > imageBorderIndices;
		currMap->addPoints( *pointCloudIn, pointIndices );
		currMap->octree_->root_->establishNeighbors();
		currMap->markNoUpdateAtPoints( *pointCloudIn, imageBorderIndices );
		currMap->evaluateSurfels();
		currMap->buildShapeTextureFeatures();

		// register scene to model
		Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

		// initialize alignment by shifting the map centroids
		Eigen::Vector3d scene_mean;
		Eigen::Matrix3d scene_cov;
		currMap->extents( scene_mean, scene_cov );

		transform.block<3,1>(0,3) = model_mean_ - scene_mean;

		pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrSrc;
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr corrTgt;

//		MultiResolutionSurfelRegistration reg;
//		reg.estimateTransformation( *map_, *currMap, transform, 32.f * currMap->min_resolution_, currMap->min_resolution_, corrSrc, corrTgt, 100, 0, 5 );

		MultiResolutionSoftSurfelRegistration reg;
		reg.estimateTransformation( *map_, *currMap, transform, 32.f * currMap->min_resolution_, currMap->min_resolution_, corrSrc, corrTgt, 100 );

		ROS_INFO_STREAM( "registering took " << sw.getTimeSeconds() );

		// visualize model
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv->header = pointCloudIn->header;
		map_->visualize3DColorDistribution( cloudv, -1, -1, false );
		pub_model_cloud.publish( cloudv );

		// visualize scene
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv2->header = pointCloudIn->header;
		currMap->visualize3DColorDistribution( cloudv2, -1, -1, false );
		pcl::transformPointCloud( *cloudv2, *cloudv2, transform );
		pub_scene_cloud.publish( cloudv2 );

		std_msgs::Int32 status;
		status.data = responseId_;
		pub_status_.publish( status );

	}

	void update() {


	}


public:

	ros::NodeHandle nh_;
	ros::Subscriber sub_cloud_;
	ros::Publisher pub_status_;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_model_cloud, pub_scene_cloud;
	boost::shared_ptr< tf::TransformListener > tf_listener_;

	bool register_map_;

	boost::shared_ptr< MultiResolutionSurfelMap > map_;
	Eigen::Vector3d model_mean_;
	Eigen::Matrix3d model_cov_;

	ros::ServiceServer register_service_;

	std::string map_folder_;
	std::string object_name_;

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

