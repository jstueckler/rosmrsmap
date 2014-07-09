/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 20.12.2011
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

#include <mrsmap/slam/slam.h>

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
#include <rosmrsmap/ObjectData.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <tf/transform_broadcaster.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <mrsmap/map/multiresolution_surfel_map.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>


#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"


#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"

#include <mrsmap/visualization/visualization_slam.h>

#include <rosmrsmap/utilities/utilities.h>




using namespace mrsmap;

boost::shared_ptr< ViewerSLAM > viewer;


class TrainObjectFromData
{
public:

    TrainObjectFromData( ros::NodeHandle& nh ) {

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );

    	// parameters

    	nh.param<std::string>( "object", object_name_, "test" );
    	nh.param<std::string>( "map_folder", map_folder_, "." );

    	nh.param<std::string>( "data_file", bag_name_, "rgbd.bag" );
    	nh.param<std::string>( "data_folder", data_folder_, "." );

    	nh.param<int>( "start_frame", start_frame_, 0 );
    	nh.param<int>( "end_frame", end_frame_, 1000 );

    	nh.param<double>( "max_resolution", min_resolution_, 0.005 );
    	nh.param<double>( "max_range", max_range_, 30.0 );
    	nh.param<double>( "dist_dep_factor", dist_dep_, 0.005 );

    	ROS_ERROR_STREAM( "max resolution: " << min_resolution_ << ", dist_dep_factor: " << dist_dep_ );

    	slam_.params_.map_dist_dependency_= dist_dep_;

    }



    void train( ros::NodeHandle& nh ) {

    	float register_start_resolution = min_resolution_;
    	const float register_stop_resolution = 32.f*min_resolution_;

    	// read data from bag file
    	rosbag::Bag bag;
		bag.open( data_folder_ + "/" + bag_name_, rosbag::bagmode::Read );

		std::vector< std::string > topics;
		topics.push_back( std::string( "object_data" ) );

		rosbag::View view( bag, rosbag::TopicQuery( topics ) );

		ROS_INFO( "reading in %i messages, using frames %i to %i", view.size(), start_frame_, end_frame_ );

		unsigned int frames = 0;
		foreach( rosbag::MessageInstance const m, view ) {

			rosmrsmap::ObjectData::ConstPtr objectData = m.instantiate< rosmrsmap::ObjectData >();
			if( objectData != NULL ) {

	    		unsigned int numEdges = slam_.optimizer_->edges().size();
				unsigned int numVertices = slam_.optimizer_->vertices().size();
				unsigned int referenceID = slam_.referenceKeyFrameId_;

				if( frames >= start_frame_ && frames <= end_frame_ ) {

					std::cout << "processing frame " << frames << "\n";

					// extract point cloud from image pair
					pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud( new pcl::PointCloud< pcl::PointXYZRGB >() );
					imagesToPointCloud( objectData->image_depth, objectData->image_rgb, cloud );

					// build map in mocap reference frame
					bool isFirstFrame = false;

					if( frames == start_frame_ ) {

						isFirstFrame = true;

					}

//					viewer->displayPointCloud( "rgbd", cloud );

					cv::Mat img_rgb;
					bool retVal = slam_.addImage( img_rgb, cloud, register_start_resolution, register_stop_resolution, min_resolution_, true );


					if( slam_.optimizer_->vertices().size() > 0 ) {
						g2o::VertexSE3* v_ref = dynamic_cast< g2o::VertexSE3* >( slam_.optimizer_->vertex( slam_.keyFrames_[ slam_.referenceKeyFrameId_ ]->nodeId_ ) );
						Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();

						viewer->displayPose( pose_ref * slam_.lastTransform_ );

					}
				}


				if( frames == end_frame_ )
					break;

				frames++;

				if( viewer->viewer->wasStopped() )
					exit(-1);

				if( slam_.optimizer_->vertices().size() != numVertices || slam_.optimizer_->edges().size() != numEdges || slam_.referenceKeyFrameId_ != referenceID )
					graphChanged = true;

				if( graphChanged || forceRedraw ) {
					viewer->visualizeSLAMGraph();
					forceRedraw = false;
					graphChanged = false;
				}

				viewer->viewer->spinOnce(1);
				usleep(10);

			}

		}

		bag.close();

//		viewer->viewer->removePointCloud( "rgbd" );

		Eigen::Matrix4d transformGuess;
		transformGuess.setIdentity();

		if( slam_.optimizer_->vertices().size() >= 3 ) {

			for( int i = 0; i < 3; i++ ) {

				slam_.connectClosePoses( register_start_resolution, register_stop_resolution );

				// optimize slam graph
				std::cout << "optimizing...\n";
				slam_.optimizer_->initializeOptimization();
				slam_.optimizer_->optimize(1);
				slam_.optimizer_->computeActiveErrors();
				std::cout << slam_.optimizer_->vertices().size() << " nodes, "
						<< slam_.optimizer_->edges().size() << " edges, "
						<< "chi2: " << slam_.optimizer_->chi2() << "\n";

			}
		}

		for( int i = 0; i < 3; i++ ) {
			slam_.connectClosePoses( register_start_resolution, register_stop_resolution );
			slam_.refine( 1, 20, register_start_resolution, register_stop_resolution );
			viewer->visualizeSLAMGraph();
			viewer->spinOnce();
			usleep(10);
		}


		Eigen::Matrix4d referenceTransform = Eigen::Matrix4d::Identity();
		boost::shared_ptr< MultiResolutionSurfelMap > graphMap = slam_.getMap( referenceTransform, min_resolution_ );

		graphMap->save( map_folder_ + "/" + object_name_ );

		while( !viewer->viewer->wasStopped() ) {

			pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
			graphMap->visualize3DColorDistribution( cloud2, -1, -1, false );
			viewer->displayPointCloud( "finalmap", cloud2 );

			viewer->spinOnce();
			usleep(2000);

		}

    }




public:

	std::string object_name_, map_folder_, bag_name_, data_folder_;

	SLAM slam_;

    boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;

    double min_resolution_, max_range_, dist_dep_;
    int start_frame_, end_frame_;

    bool graphChanged, forceRedraw;

};



int main(int argc, char** argv) {

	ros::init(argc, argv, "~");
	ros::NodeHandle n("~");
	TrainObjectFromData tofd( n );

	viewer = boost::shared_ptr< ViewerSLAM >( new ViewerSLAM( &tofd.slam_ ) );

	tofd.train( n );


	while( !viewer->viewer->wasStopped() ) {

		if( tofd.graphChanged || tofd.forceRedraw ) {
			viewer->visualizeSLAMGraph();
			tofd.forceRedraw = false;
			tofd.graphChanged = false;
		}

		viewer->spinOnce();
		usleep(1000);
	}


	return 0;
}

