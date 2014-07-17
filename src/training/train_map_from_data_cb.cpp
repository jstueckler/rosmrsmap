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




boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


int selectedDepth = 11;
int selectedViewDir = -1;
bool displayRandom = false;
bool displayAll = true;
bool displayMap = true;
bool integrateMeasurements = true;
bool graphChanged = true;
bool forceRedraw = false;


using namespace mrsmap;


class ChessboardInfo {
public:

	ChessboardInfo() {
		initPose();
	}

	~ChessboardInfo() {}

	void initPose() {
		cv::Mat R = cv::Mat::eye( 3, 3, CV_64FC1 );
		cv::Rodrigues( R, rotation );
		translation = cv::Mat::zeros( 3, 1, CV_64FC1 );
		trackInitialized = false;
	}

	void initCorners() {

		corners.clear();
		for( int i = 0; i < size.height; i++ )
			for( int j = 0; j < size.width; j++ )
				corners.push_back( cv::Point3f( float(j*squareSize), float(i*squareSize), 0.f ) );

	}

	cv::Size size;
	double squareSize;
	std::vector< cv::Point3f > corners;
	cv::Mat rotation, translation;
	bool trackInitialized;

};


struct MouseEvent {

    MouseEvent() { event = -1; buttonState = 0; }
    cv::Point pt;
    int event;
    int buttonState;

};

static void onMouse(int event, int x, int y, int flags, void* userdata) {
	if( userdata ) {
		MouseEvent* data = (MouseEvent*)userdata;
		data->event = event;
		data->pt = cv::Point(x,y);
		data->buttonState = flags;
	}
}


class PoseInfo {
public:
	PoseInfo( const std::string& time, int id, const Eigen::Matrix4d tf )
	: stamp( time ), referenceID( id ), transform( tf ) {}
	~PoseInfo() {}

	std::string stamp;
	int referenceID;
	Eigen::Matrix4d transform;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class TrainObjectFromData
{
public:

    TrainObjectFromData( ros::NodeHandle& nh ) {

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );

    	// parameters

    	nh.param<std::string>( "object", object_name_, "test" );
    	nh.param<std::string>( "bag", bag_name_, "testdata.bag" );
    	nh.param<int>( "start_frame", start_frame_, 0 );
    	nh.param<int>( "end_frame", end_frame_, 1000 );
    	nh.param<bool>( "load_map", load_map_, false );
    	nh.param<bool>( "low_motion", low_motion_, false );

    	nh.param<bool>( "use_cb", use_cb_, false );


    	nh.param<double>( "min_height", minHeight_, 0.1 );
    	nh.param<double>( "max_height", maxHeight_, 2.0 );

    	nh.param<double>( "min_resolution", min_resolution_, 0.05 );
    	nh.param<double>( "max_range", max_range_, 30.0 );

    	double tx,ty,tz,qx,qy,qz,qw;
    	nh.param<double>( "mocap_camdiff_t_x", tx, 0.0 );
    	nh.param<double>( "mocap_camdiff_t_y", ty, 0.0 );
    	nh.param<double>( "mocap_camdiff_t_z", tz, 0.0 );
    	nh.param<double>( "mocap_camdiff_q_x", qx, 0.0 );
    	nh.param<double>( "mocap_camdiff_q_y", qy, 0.0 );
    	nh.param<double>( "mocap_camdiff_q_z", qz, 0.0 );
    	nh.param<double>( "mocap_camdiff_q_w", qw, 1.0 );

    	T_mocapcam2opticcam.setIdentity();
    	T_mocapcam2opticcam.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( qw, qx, qy, qz ) );
    	T_mocapcam2opticcam(0,3) = tx;
    	T_mocapcam2opticcam(1,3) = ty;
    	T_mocapcam2opticcam(2,3) = tz;

    	T_mocapcam2opticcam_tf.setOrigin( tf::Vector3( tx, ty, tz ) );
		T_mocapcam2opticcam_tf.setRotation( tf::Quaternion( qx, qy, qz, qw ) );


		if( use_cb_ ) {

			ChessboardInfo cbInfo;

			nh.param( "board_width", cbInfo.size.width, 5 );
			nh.param( "board_height", cbInfo.size.height, 4 );
			nh.param( "board_square_size", cbInfo.squareSize, 0.02 );

			ROS_INFO( "defined board: %ix%i, square size %f", cbInfo.size.width, cbInfo.size.height, cbInfo.squareSize );

			cbInfo.initCorners();

			chessboard_ = cbInfo;

		}


    }

    void displayPointCloud( const std::string& name, const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud ) {

    	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
    	pcl::copyPointCloud( *cloud, *cloud2 );

    	for( int i = 0; i < cloud2->points.size(); i++ )
    		if( isnan( cloud2->points[i].x ) ) {
    			cloud2->points[i].x = 0;
    			cloud2->points[i].y = 0;
    			cloud2->points[i].z = 0;
    		}

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>( cloud2 );
		viewer->removePointCloud( name );
		viewer->addPointCloud<pcl::PointXYZRGB>( cloud2, rgb, name );

    }


    void displayPose( const Eigen::Matrix4d& pose ) {

    	static int poseidx = 0;

		double axislength = 0.2;

		pcl::PointXYZRGB p1, p2;

		char str[255];

		sprintf( str, "posex%i", poseidx-1 );
		viewer->removeShape( str );
		sprintf( str, "posex%i", poseidx );
		p1.x = pose(0,3);
		p1.y = pose(1,3);
		p1.z = pose(2,3);
		p2.x = p1.x + axislength*pose(0,0);
		p2.y = p1.y + axislength*pose(1,0);
		p2.z = p1.z + axislength*pose(2,0);
		viewer->addLine( p1, p2, 1.0, 0.0, 0.0, str );
		viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, str );


		sprintf( str, "posey%i", poseidx-1 );
		viewer->removeShape( str );
		sprintf( str, "posey%i", poseidx );
		p1.x = pose(0,3);
		p1.y = pose(1,3);
		p1.z = pose(2,3);
		p2.x = p1.x + axislength*pose(0,1);
		p2.y = p1.y + axislength*pose(1,1);
		p2.z = p1.z + axislength*pose(2,1);
		viewer->addLine( p1, p2, 0.0, 1.0, 0.0, str );
		viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, str );



		sprintf( str, "posez%i", poseidx-1 );
		viewer->removeShape( str );
		sprintf( str, "posez%i", poseidx );
		p1.x = pose(0,3);
		p1.y = pose(1,3);
		p1.z = pose(2,3);
		p2.x = p1.x + axislength*pose(0,2);
		p2.y = p1.y + axislength*pose(1,2);
		p2.z = p1.z + axislength*pose(2,2);
		viewer->addLine( p1, p2, 0.0, 0.0, 1.0, str );
		viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, str );

		poseidx++;


    }


	void visualizeSLAMGraph() {


		for( unsigned int i = 0; i < currShapes.size(); i++ ) {

			char str[255];
			sprintf(str,"%i",currShapes[i]);
			viewer->removeShape( str );

		}

		currShapes.clear();

		for( unsigned int i = 0; i < slam_.keyFrames_.size(); i++ ) {

			g2o::VertexSE3* v_curr = dynamic_cast< g2o::VertexSE3* >( slam_.optimizer_->vertex( i ) );
			char str[255];

			// add coordinate frames for vertices
			Eigen::Matrix4d camTransform = v_curr->estimate().matrix();

			if( i == slam_.referenceKeyFrameId_ || displayAll ) {

				double axislength = 0.2;
				double linewidth = 5;

				if( i == slam_.referenceKeyFrameId_ )
					linewidth = 10, axislength = 0.5;

				pcl::PointXYZRGB p1, p2;

				p1.x = camTransform(0,3);
				p1.y = camTransform(1,3);
				p1.z = camTransform(2,3);
				p2.x = p1.x + axislength*camTransform(0,0);
				p2.y = p1.y + axislength*camTransform(1,0);
				p2.z = p1.z + axislength*camTransform(2,0);
				sprintf(str,"%i",shapeIdx);
				viewer->addLine( p1, p2, 1.0, 0.0, 0.0, std::string( str ) );
				viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, str );
				currShapes.push_back( shapeIdx++ );

				p2.x = p1.x + axislength*camTransform(0,1);
				p2.y = p1.y + axislength*camTransform(1,1);
				p2.z = p1.z + axislength*camTransform(2,1);
				sprintf(str,"%i",shapeIdx);
				viewer->addLine( p1, p2, 0.0, 1.0, 0.0, std::string( str ) );
				viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, str );
				currShapes.push_back( shapeIdx++ );

				p2.x = p1.x + axislength*camTransform(0,2);
				p2.y = p1.y + axislength*camTransform(1,2);
				p2.z = p1.z + axislength*camTransform(2,2);
				sprintf(str,"%i",shapeIdx);
				viewer->addLine( p1, p2, 0.0, 0.0, 1.0, std::string( str ) );
				viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, str );
				currShapes.push_back( shapeIdx++ );

			}

			if( displayAll && displayMap ) {

				if( slam_.optimizer_->vertices()[i]->edges().size() > 0 ) {

					// only revisualize maps if current pose is "close"
					g2o::VertexSE3* v_ref = dynamic_cast< g2o::VertexSE3* >( slam_.optimizer_->vertex( slam_.referenceKeyFrameId_ ) );
					Eigen::Matrix4d curr_pose = v_ref->estimate().matrix() * slam_.lastTransform_;

					Eigen::Matrix4d diffTransform = camTransform.inverse() * curr_pose;

//					Eigen::AngleAxisd q( diffTransform.block<3,3>(0,0) );

					if( diffTransform.block<3,1>(0,3).norm() < 2.0 ) {

						// visualize maps at estimated pose
						pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
						slam_.keyFrames_[i]->map_->visualize3DColorDistribution( cloud2, selectedDepth, selectedViewDir, displayRandom );

						pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedCloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
						pcl::transformPointCloud( *cloud2, *transformedCloud, camTransform.cast<float>() );

						sprintf(str,"map%i",i);
						displayPointCloud( str, transformedCloud );

					}


				}
				else {
					sprintf(str,"map%i",i);
					viewer->removePointCloud( str );
				}
			}
			else {
				sprintf(str,"map%i",i);
				viewer->removePointCloud( str );
			}

		}


		if( displayAll ) {

			for( EdgeSet::iterator it = slam_.optimizer_->edges().begin(); it != slam_.optimizer_->edges().end(); ++it ) {

				g2o::EdgeSE3* edge = dynamic_cast< g2o::EdgeSE3* >( *it );

				// add lines for edges
				g2o::VertexSE3* v1 = dynamic_cast< g2o::VertexSE3* >( edge->vertices()[0] );
				g2o::VertexSE3* v2 = dynamic_cast< g2o::VertexSE3* >( edge->vertices()[1] );

				// add coordinate frames for vertices
				Eigen::Matrix4d pose1 = v1->estimate().matrix();
				Eigen::Matrix4d pose2 = v2->estimate().matrix();

				pcl::PointXYZRGB p1, p2;
				char str[255];
				sprintf(str,"%i",shapeIdx);

				p1.x = pose1(0,3);
				p1.y = pose1(1,3);
				p1.z = pose1(2,3);
				p2.x = pose2(0,3);
				p2.y = pose2(1,3);
				p2.z = pose2(2,3);

				viewer->addLine( p1, p2, 0.0, 0.0, 0.0, std::string( str ) );
				viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, str );
				currShapes.push_back( shapeIdx++ );

			}
		}

	}


    void imagesToPointCloud( const sensor_msgs::Image& depthImg, const sensor_msgs::Image colorImg, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud ) {

    	cloud->header = pcl_conversions::toPCL(depthImg.header);
    	cloud->is_dense = true;
    	cloud->height = depthImg.height;
    	cloud->width = depthImg.width;
    	cloud->sensor_origin_ = Eigen::Vector4f( 0.f, 0.f, 0.f, 0.f );
    	cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    	cloud->points.resize( depthImg.width*depthImg.height );

		const float invfocalLength = 1.f / 525.f;
		const float centerX = 319.5f;
		const float centerY = 239.5f;

    	const float* depthdata = reinterpret_cast<const float*>( &depthImg.data[0] );
    	const unsigned char* colordata = &colorImg.data[0];
    	int idx = 0;
    	for( unsigned int y = 0; y < depthImg.height; y++ ) {
    		for( unsigned int x = 0; x < depthImg.width; x++ ) {

        		pcl::PointXYZRGB& p = cloud->points[idx];

				float dist = (*depthdata);

        		if( isnan( dist ) ) {
        			p.x = std::numeric_limits<float>::quiet_NaN();
        			p.y = std::numeric_limits<float>::quiet_NaN();
        			p.z = std::numeric_limits<float>::quiet_NaN();
        		}
        		else {

					float xf = x;
					float yf = y;
					p.x = (xf-centerX) * dist * invfocalLength;
					p.y = (yf-centerY) * dist * invfocalLength;
					p.z = dist;
        		}

        		depthdata++;

				int r = (*colordata++);
				int g = (*colordata++);
				int b = (*colordata++);

    			int rgb = ( r << 16 ) + ( g << 8 ) + b;
    			p.rgb = * ( reinterpret_cast< float* > ( &rgb ) );

    			idx++;


    		}
    	}

    }


    bool selectConvexHull( const rosmrsmap::ObjectData::ConstPtr& objectData, const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud, tf::Transform referenceTransform ) {

		// let user select convex hull points in the images
		ROS_INFO( "select convex hull in the image" );
		ROS_INFO( "left click: add point, right click: finish selection" );

		sensor_msgs::Image::Ptr image = sensor_msgs::Image::Ptr( new sensor_msgs::Image( objectData->image_rgb ) );
		sensor_msgs::CameraInfo::ConstPtr cameraInfo = fillCameraInfo( objectData->header.stamp, objectData->image_rgb.width, objectData->image_rgb.height );

		// convert to opencv image
		cv::Mat img_cv;
		cv_bridge::CvImagePtr cvimgptr;
		try {
			cvimgptr = cv_bridge::toCvCopy( image, "mono8" );
			img_cv = cvimgptr->image;
		} catch( const cv_bridge::Exception& error ) {
			ROS_ERROR("CvBridge: %s", error.what());
		}

		// extract camera model
		image_geometry::PinholeCameraModel cameraModel;
		cameraModel.fromCameraInfo( cameraInfo );


		// wait for user input
		// left click: save clicked point in reference frame
		// right/middle click: stop selection

		bool stopSelection = false;
		while( !stopSelection ) {

			// project selected points into image
			cv::Mat img_viz;
			cv::cvtColor( img_cv, img_viz, CV_GRAY2BGR );
			if( convexHull_.size() > 0 ) {

				std::vector< cv::Point3f > convexHullCamera( convexHull_.size() );
				for( unsigned int j = 0; j < convexHull_.size(); j++ ) {

					// transform point from reference frame to camera frame
					tf::Point tfp;
					tfp[0] = convexHull_[j](0);
					tfp[1] = convexHull_[j](1);
					tfp[2] = convexHull_[j](2);
					tfp[3] = 1;

					tfp = referenceTransform.inverse() * tfp;
					convexHullCamera[j].x = tfp[0];
					convexHullCamera[j].y = tfp[1];
					convexHullCamera[j].z = tfp[2];

				}

				std::vector< cv::Point2f > imagePoints(convexHullCamera.size());
				cv::Mat rot( 3, 1, CV_64FC1, 0.f );
				cv::Mat trans( 3, 1, CV_64FC1, 0.f );
				cv::projectPoints( cv::Mat(convexHullCamera), rot, trans, cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(), imagePoints );

				for( unsigned int j = 0; j < imagePoints.size(); j++ ) {

					if( imagePoints[j].x < 0 || imagePoints[j].x > image->width-1 )
						continue;

					if( imagePoints[j].y < 0 || imagePoints[j].y > image->height-1 )
						continue;

					cv::Scalar c( 0.f, 0.f, 255.f );
					cv::circle( img_viz, imagePoints[j], 10, c, -1 );

				}
			}

			int displayHeight = 240;
			double imgScaleFactor = ((float)displayHeight) / ((float)img_viz.rows);
			if( img_viz.rows != displayHeight ) {
				cv::Mat tmp;
				cv::resize( img_viz, tmp, cv::Size(), imgScaleFactor, imgScaleFactor, cv::INTER_LINEAR );
				tmp.copyTo( img_viz );
			}
			cv::imshow( "Select Convex Hull", img_viz );

			MouseEvent mouse;
			cv::setMouseCallback( "Select Convex Hull", onMouse, &mouse );
			cv::waitKey(10);

			if( mouse.event == CV_EVENT_LBUTTONDOWN ) {

				// find corresponding 3D position in point cloud
				float img2cloudScale = ((float)cloud->height) / ((float)displayHeight);
				unsigned int idx = round(img2cloudScale * ((float)mouse.pt.y)) * cloud->width + round(img2cloudScale * ((float)mouse.pt.x));
				if( idx < cloud->points.size() && !isnan(cloud->points[idx].x) ) {

					//  transform point to reference frame
					tf::Point tfp;
					tfp[0] = cloud->points[idx].x;
					tfp[1] = cloud->points[idx].y;
					tfp[2] = cloud->points[idx].z;
					tfp[3] = 1;

					tfp = referenceTransform * tfp;

					Eigen::Vector3f p( tfp[0], tfp[1], tfp[2] );
					convexHull_.push_back( p );
				}

			}
			else if( mouse.event == CV_EVENT_RBUTTONDOWN ) {
				stopSelection = true;
			}
			else if( mouse.event == CV_EVENT_MBUTTONDOWN ) {
				stopSelection = true;
			}
		}

		if( convexHull_.size() < 3 ) {
			ROS_ERROR("convex hull requires more than 3 points");
			return false;
		}
		else {

			// project selected points on common plane

			Eigen::Vector4f plane_parameters;

			// Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
			EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
			Eigen::Vector4f xyz_centroid;

			pcl::PointCloud< pcl::PointXYZ >::Ptr selectedPoints( new pcl::PointCloud< pcl::PointXYZ >() );
			for( unsigned int i = 0; i < convexHull_.size(); i++ ) {
				pcl::PointXYZ p;
				p.x = convexHull_[i](0);
				p.y = convexHull_[i](1);
				p.z = convexHull_[i](2);
				selectedPoints->points.push_back( p );
			}

			// Estimate the XYZ centroid
			pcl::compute3DCentroid( *selectedPoints, xyz_centroid );
			xyz_centroid[3] = 0;

			// Compute the 3x3 covariance matrix
			pcl::computeCovarianceMatrix( *selectedPoints, xyz_centroid, covariance_matrix );

			// Compute the model coefficients
			EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
			EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
			pcl::eigen33( covariance_matrix, eigen_vectors, eigen_values );

			// remove components orthogonal to the plane..
			for( unsigned int i = 0; i < convexHull_.size(); i++ ) {

				Eigen::Vector3f p = convexHull_[i];

				float l = p.dot( eigen_vectors.block<3,1>(0,0) ) - xyz_centroid.block<3,1>(0,0).dot( eigen_vectors.block<3,1>(0,0) );

				p -= l * eigen_vectors.block<3,1>(0,0);

				convexHull_[i](0) = p(0);
				convexHull_[i](1) = p(1);
				convexHull_[i](2) = p(2);

			}

		}

		return true;

    }

    sensor_msgs::CameraInfoPtr fillCameraInfo( ros::Time time, unsigned int width, unsigned int height ) {

		sensor_msgs::CameraInfoPtr info_msg = boost::make_shared< sensor_msgs::CameraInfo >();
		info_msg->header.stamp    = time;
		info_msg->header.frame_id = "openni_rgb_optical_frame";
		info_msg->width           = width;
		info_msg->height          = height;

		info_msg->D = std::vector< double >( 5, 0.0 );
		info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		info_msg->K.assign( 0.0 );
		info_msg->R.assign( 0.0 );
		info_msg->P.assign( 0.0 );

		// Simple camera matrix: square pixels, principal point at center
		double f = 1050.f * (float)width / 1280.f;
		info_msg->K[0] = info_msg->K[4] = f;
		info_msg->K[2] = (info_msg->width / 2) - 0.5;
		info_msg->K[5] = (info_msg->width * 3./8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
		info_msg->K[8] = 1.0;
		// no rotation: identity
		info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
		// no rotation, no translation => P=K(I|0)=(K|0)
		info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
		info_msg->P[2] = info_msg->K[2];
		info_msg->P[6] = info_msg->K[5];
		info_msg->P[10] = 1.0;

		return info_msg;
    }

    void train( ros::NodeHandle& nh ) {

    	referenceTransform_.setIdentity();

    	float register_start_resolution = min_resolution_;
    	const float register_stop_resolution = 32.f*min_resolution_;

    	std::vector< PoseInfo, Eigen::aligned_allocator< PoseInfo > > trajectoryEstimate;
    	std::vector< PoseInfo, Eigen::aligned_allocator< PoseInfo > > trajectoryGT;

    	// read data from bag file
    	rosbag::Bag bag;
		bag.open( bag_name_, rosbag::bagmode::Read );

		std::vector< std::string > topics;
		topics.push_back( std::string( "object_data" ) );

		rosbag::View view( bag, rosbag::TopicQuery( topics ) );

		ROS_INFO( "reading in %i messages, using frames %i to %i", view.size(), start_frame_, end_frame_ );

		unsigned int frames = 0;
		foreach( rosbag::MessageInstance const m, view ) {

			rosmrsmap::ObjectData::ConstPtr objectData = m.instantiate< rosmrsmap::ObjectData >();
			if( objectData != NULL ) {

				if( frames >= start_frame_ && frames <= end_frame_ ) {

					std::cout << "processing frame " << frames << "\n";

					// extract point cloud from image pair
					pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud( new pcl::PointCloud< pcl::PointXYZRGB >() );
					imagesToPointCloud( objectData->image_depth, objectData->image_rgb, cloud );


					// mocap camera pose in mocap frame
					tf::Transform currMocapCamPose;
					tf::poseMsgToTF( objectData->mocap_camera_pose.pose, currMocapCamPose );
					currMocapCamPose = currMocapCamPose.inverse();
					tf::Transform mocapTransform = currMocapCamPose * T_mocapcam2opticcam_tf.inverse();

					// handle missing mocap information
					if( fabsf(objectData->mocap_camera_pose.pose.position.x) <= std::numeric_limits<float>::epsilon() && fabsf(objectData->mocap_camera_pose.pose.position.y) <= std::numeric_limits<float>::epsilon() && fabsf(objectData->mocap_camera_pose.pose.position.z) <= std::numeric_limits<float>::epsilon() )
						mocapTransform.setIdentity();


					// build map in mocap reference frame
					bool isFirstFrame = false;

					if( !use_cb_ ) {

						if( frames == start_frame_ ) {

							// optical camera pose in mocap frame
							// => fuse data in mocap frame
							referenceTransform_ = mocapTransform;

							isFirstFrame = true;

						}

					}
					else {


						if( !chessboard_.trackInitialized ) {


							// detect cb in image..

							// convert to opencv image
							sensor_msgs::Image::Ptr image = sensor_msgs::Image::Ptr( new sensor_msgs::Image( objectData->image_rgb ) );

							cv::Mat img_cv;
							cv_bridge::CvImagePtr cvimgptr;
							try {
								cvimgptr = cv_bridge::toCvCopy( image, "mono8" );
								img_cv = cvimgptr->image;
							} catch( const cv_bridge::Exception& error ) {
								ROS_ERROR("CvBridge: %s", error.what());
							}

//							sensor_msgs::CvBridge cvBridge;
//							cv::Mat img_cv;
//							try {
//								img_cv = cvBridge.imgMsgToCv( image, "mono8" );
//							} catch (const sensor_msgs::CvBridgeException& error) {
//								ROS_ERROR("CvBridge: %s", error.what());
//							}

							sensor_msgs::CameraInfo::ConstPtr cameraInfo = fillCameraInfo( objectData->header.stamp, objectData->image_rgb.width, objectData->image_rgb.height );
							image_geometry::PinholeCameraModel cameraModel;
							cameraModel.fromCameraInfo( cameraInfo );

							bool boardFound = false;

							std::vector< cv::Point2f > foundBoardCorners;
							boardFound = findChessboardCorners( img_cv, chessboard_.size, foundBoardCorners, cv::CALIB_CB_ADAPTIVE_THRESH );
							if( boardFound ) {

								ROS_INFO( "found board" );
								cv::cornerSubPix( img_cv, foundBoardCorners, cv::Size( 5, 5 ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_ITER, 20, 1e-2 ) );

//								// draw found corners..
//								cv::drawChessboardCorners( img_viz, chessboard_.size, cv::Mat( foundBoardCorners ), true );

								// extract chessboard pose, use last pose as initial guess
								cv::Mat last_rotation( 3, 1, CV_64FC1 );
								chessboard_.rotation.copyTo( last_rotation );
								cv::Mat last_translation( 3, 1, CV_64FC1 );
								chessboard_.translation.copyTo( last_translation );

								cv::solvePnP( cv::Mat( chessboard_.corners ), cv::Mat( foundBoardCorners ), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(), chessboard_.rotation, chessboard_.translation, false );

								chessboard_.trackInitialized = true;

								cv::Mat R( 3, 3, CV_64FC1 );
								cv::Rodrigues( chessboard_.rotation, R );
								tf::Matrix3x3 matR;
								tf::Vector3 t;


								for( int y = 0; y < 3; y++ ) {
									for( int x = 0; x < 3; x++ )
										matR[y][x] = R.at<double>(y,x);
									t[y] = chessboard_.translation.at<double>(y,0);
								}

								referenceTransform_ = tf::Transform( matR, t );

								tf::Transform fixTransform;
								fixTransform.setIdentity();
								tf::Quaternion fixRot;
								fixRot.setRotation( tf::Vector3( 1.0, 0.0, 0.0 ), M_PI );
								fixTransform.setRotation( fixRot );

								referenceTransform_ = referenceTransform_.inverse();

								referenceTransform_ = fixTransform * referenceTransform_;

								for( int y = 0; y < 3; y++ ) {
									for( int x = 0; x < 3; x++ )
										std::cout << referenceTransform_.getBasis()[y][x] << "\n";
								}

								isFirstFrame = true;


							}

						}

					}


					if( isFirstFrame ) {

						selectConvexHull( objectData, cloud, referenceTransform_ );

//						slam_.lastTransform_.setIdentity();
//						slam_.lastTransform_.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( referenceTransform_.getRotation().w(), referenceTransform_.getRotation().x(), referenceTransform_.getRotation().y(), referenceTransform_.getRotation().z() ) );
//						slam_.lastTransform_(0,3) = referenceTransform_.getOrigin().x();
//						slam_.lastTransform_(1,3) = referenceTransform_.getOrigin().y();
//						slam_.lastTransform_(2,3) = referenceTransform_.getOrigin().z();
//
//						pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedCloud( new pcl::PointCloud< pcl::PointXYZRGB >() );
//						pcl_ros::transformPointCloud< pcl::PointXYZRGB >( *cloud, *transformedCloud, referenceTransform_ );
//						*cloud = *transformedCloud;

					}

					if( !use_cb_ || chessboard_.trackInitialized ) {

						int numEdges = slam_.optimizer_->edges().size();
						int numVertices = slam_.optimizer_->vertices().size();
						int referenceID = slam_.referenceKeyFrameId_;

	//					displayPointCloud("test", cloud);

						cv::Mat img_rgb;
						bool retVal = slam_.addImage( img_rgb, cloud, register_start_resolution, register_stop_resolution, min_resolution_, true );
	//					bool retVal = slam_.addImage( cloud, register_start_resolution, register_stop_resolution, min_resolution_, false );

						slam_.refineWorstEdges( 0.05, register_start_resolution, register_stop_resolution );

						if( retVal ) {

							std::stringstream sstr;
							sstr << std::fixed << std::setprecision(10) << objectData->header.stamp.toSec();
							trajectoryEstimate.push_back( PoseInfo( sstr.str(), slam_.referenceKeyFrameId_, slam_.lastTransform_ ) );

						}

						if( slam_.optimizer_->vertices().size() != numVertices || slam_.optimizer_->edges().size() != numEdges || slam_.referenceKeyFrameId_ != referenceID )
							graphChanged = true;

						if( slam_.optimizer_->vertices().size() > 0 ) {
							g2o::VertexSE3* v_ref = dynamic_cast< g2o::VertexSE3* >( slam_.optimizer_->vertex( slam_.referenceKeyFrameId_ ) );
							Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();

							displayPose( pose_ref * slam_.lastTransform_ );


	//						pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedCloud( new pcl::PointCloud< pcl::PointXYZRGB >() );
	//						pcl::transformPointCloud< pcl::PointXYZRGB >( *cloud, *transformedCloud, (pose_ref * slam_.lastTransform_).cast<float>() );
	//
	//						displayPointCloud( "currentframe", transformedCloud );


	//						viewer->updatePointCloud( transformedCloud, "current frame" );

						}

						if( !use_cb_ )
						{
							std::stringstream sstr;
							sstr << std::fixed << std::setprecision(10) << objectData->header.stamp.toSec();

							Eigen::Matrix4d transform;
							transform.setIdentity();
							transform.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( mocapTransform.getRotation().w(), mocapTransform.getRotation().x(), mocapTransform.getRotation().y(), mocapTransform.getRotation().z() ) );
							transform(0,3) = mocapTransform.getOrigin().x();
							transform(1,3) = mocapTransform.getOrigin().y();
							transform(2,3) = mocapTransform.getOrigin().z();

							trajectoryGT.push_back( PoseInfo( sstr.str(), -1, transform ) );
						}
					}


				}

				if( frames == end_frame_ )
					break;

				frames++;

//				while( true ) {
//
//					if( nextFrame ) {
//						nextFrame = false;
//						break;
//					}

					if( viewer->wasStopped() )
						exit(-1);

					if( graphChanged || forceRedraw ) {
						visualizeSLAMGraph();
						forceRedraw = false;
						graphChanged = false;
					}

					viewer->spinOnce(1);
					usleep(10);
//				}
			}

		}

		bag.close();

		Eigen::Matrix4d transformGuess;
		transformGuess.setIdentity();
//		slam_.addEdge( 0, slam_.keyFrames_.size()-1, transformGuess, register_start_resolution, register_stop_resolution );

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


//		slam_.dumpError();

		Eigen::Matrix4d refTransform;
		refTransform.setIdentity();
		refTransform.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( referenceTransform_.getRotation().w(), referenceTransform_.getRotation().x(), referenceTransform_.getRotation().y(), referenceTransform_.getRotation().z() ) );
		refTransform(0,3) = referenceTransform_.getOrigin().x();
		refTransform(1,3) = referenceTransform_.getOrigin().y();
		refTransform(2,3) = referenceTransform_.getOrigin().z();

//		slam_.refine( 0, 1000, register_start_resolution, register_stop_resolution );
		for( int i = 0; i < 3; i++ ) {
			slam_.connectClosePoses( register_start_resolution, register_stop_resolution );
//			slam_.refineInConvexHull( 1, 20, register_start_resolution, register_stop_resolution, min_resolution_, refTransform, minHeight_, maxHeight_, convexHull_ );
			slam_.refine( 1, 20, register_start_resolution, register_stop_resolution );
			visualizeSLAMGraph();
			viewer->spinOnce(1);
			usleep(10);
		}



//			slam_.refine( 1, 100, register_start_resolution, register_stop_resolution );
//			visualizeSLAMGraph();
//			viewer->spinOnce(1);
//			usleep(10);
//		}

		std::ofstream fileEstimate( "pose_estimate.txt" );
		std::ofstream fileGroundTruth( "pose_groundtruth.txt" );

		// dump pose ground truth to file
		for( int i = 0; i < trajectoryGT.size(); i++ ) {

			Eigen::Matrix4d transform = trajectoryGT[i].transform;

			// check sanity of GT transform (there could be mocap errors)

			// check for identity transforms (mocap missing)
			if( transform.block<3,1>(0,3).norm() <= 0.01 )
				continue;

			// check for big jumps to the next frame
			if( i < trajectoryGT.size()-1 ) {

				// nothing to compare with?
				Eigen::Matrix4d transform_ip1 = trajectoryGT[i+1].transform;
				if(  transform_ip1.block<3,1>(0,3).norm() > 0.01 ) {

					Eigen::Matrix4d diffTransform = transform.inverse() * transform_ip1;
					if( Eigen::AngleAxisd( diffTransform.block<3,3>(0,0) ).angle() > 0.5f || diffTransform.block<3,1>(0,3).norm() > 0.2f )
						continue;

				}
				else
					continue;

			}

			Eigen::Quaterniond q( Eigen::Matrix3d( transform.block<3,3>(0,0) ) );
			fileGroundTruth << trajectoryGT[i].stamp << " " << transform(0,3) << " " << transform(1,3) << " " << transform(2,3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

		}


		// dump pose estimates to file
		for( int i = 0; i < trajectoryEstimate.size(); i++ ) {

			g2o::VertexSE3* v_curr = dynamic_cast< g2o::VertexSE3* >( slam_.optimizer_->vertex( trajectoryEstimate[i].referenceID ) );
			Eigen::Matrix4d vtransform = v_curr->estimate().matrix();

			Eigen::Matrix4d transform = vtransform * trajectoryEstimate[i].transform;

			Eigen::Quaterniond q( Eigen::Matrix3d( transform.block<3,3>(0,0) ) );
			fileEstimate << trajectoryEstimate[i].stamp << " " << transform(0,3) << " " << transform(1,3) << " " << transform(2,3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

		}

		fileEstimate.close();
		fileGroundTruth.close();


		boost::shared_ptr< MultiResolutionSurfelMap > graphMap = slam_.getMapInConvexHull( refTransform, min_resolution_, minHeight_, maxHeight_, convexHull_ );

		for( unsigned int i = 0; i < slam_.keyFrames_.size(); i++ ) {
			char str[255];
			sprintf(str,"map%i",i);
			viewer->removePointCloud( str );
		}


		graphMap->save(ros::package::getPath("nimbro_launch") + "/objectknowledge/" + object_name_+".map");

		while( !viewer->wasStopped() ) {

			pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2 = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
			graphMap->visualize3DColorDistribution( cloud2, selectedDepth, selectedViewDir, displayRandom );
			displayPointCloud( "finalmap", cloud2 );
			std::cout << "points in final map " << cloud2->points.size() << "\n";

			viewer->spinOnce(1);
			usleep(2000);
		}

    }




public:

	std::string bag_name_;
	std::string object_name_;
    pcl::PointCloud< pcl::PointXYZRGB > stitchedCloud_;
    double minHeight_, maxHeight_;

    std::vector< Eigen::Vector3f, Eigen::aligned_allocator< Eigen::Vector3f > > convexHull_;

    tf::Transform referenceTransform_;
	SLAM slam_;

	int shapeIdx;
	std::vector< int > currShapes;

    Eigen::Matrix4d T_mocapcam2opticcam;
    tf::Transform T_mocapcam2opticcam_tf;

    boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;

    double min_resolution_, max_range_;
    int start_frame_, end_frame_;
    bool load_map_;
    bool low_motion_;

    bool use_cb_; // use checkerboard instead of mocap for first frame
    ChessboardInfo chessboard_;

};


void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &event, void* viewer_void ) {
	TrainObjectFromData* ev = (TrainObjectFromData*) viewer_void;

	if( (event.getKeySym() == "d" || event.getKeySym() == "D") && event.keyDown() ) {

		if( event.getKeySym() == "d" ) {
			selectedDepth++;
		}
		else {
			selectedDepth--;
			if( selectedDepth < 0 )
				selectedDepth = 15;
		}

		selectedDepth = selectedDepth % 16;
		std::cout << "Selected Depth " << selectedDepth << "\n";
	}
	if( (event.getKeySym() == "v" || event.getKeySym() == "V") && event.keyDown() ) {

		if( event.getKeySym() == "v" ) {
			selectedViewDir++;
			if( selectedViewDir == 7 )
				selectedViewDir = -1;
		}
		else {
			selectedViewDir--;
			if( selectedViewDir < -1 )
				selectedViewDir = 6;
		}

		std::cout << "Selected View Dir " << selectedViewDir << "\n";

		viewer->removeShape( "selected viewdir" );
		if( selectedViewDir >= 0 ) {
			MultiResolutionSurfelMap::NodeValue v;
			v.initialize();
			Eigen::Vector3d dir = v.surfels_[selectedViewDir].initial_view_dir_;
			pcl::PointXYZRGB p1, p2;
			p1.x = p1.y = p1.z = 0;
			p2.x = dir(0);
			p2.y = dir(1);
			p2.z = dir(2);
			viewer->addArrow( p1, p2, 1, 0, 0, "selected viewdir" );
		}
	}
	if( (event.getKeySym() == "m") && event.keyDown() ) {
		displayMap = !displayMap;
	}
	if( (event.getKeySym() == "a") && event.keyDown() ) {
		displayAll = !displayAll;
	}
	if( (event.getKeySym() == "r") && event.keyDown() ) {
		displayRandom = !displayRandom;
	}
	if( (event.getKeySym() == "i") && event.keyDown() ) {
		integrateMeasurements = !integrateMeasurements;
	}
	if( (event.getKeySym() == "f") && event.keyDown() ) {
		forceRedraw = true;
	}

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "~");
	ros::NodeHandle n("~");
	TrainObjectFromData tofd( n );

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>( new pcl::visualization::PCLVisualizer( "3D Viewer" ) );
	viewer->setBackgroundColor( 1, 1, 1 );
	viewer->addCoordinateSystem( 1.0 );
	viewer->initCameraParameters();

	viewer->registerKeyboardCallback( keyboardEventOccurred, (void*)&tofd );

//	boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > > cloudPtr = boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGB > >( new pcl::PointCloud< pcl::PointXYZRGB >() );
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>( cloudPtr );
//	viewer->addPointCloud<pcl::PointXYZRGB>( cloudPtr, rgb, "current frame" );
//	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current frame" );


	tofd.train( n );


	while( !viewer->wasStopped() ) {

		if( graphChanged || forceRedraw ) {
			tofd.visualizeSLAMGraph();
			forceRedraw = false;
			graphChanged = false;
		}

		viewer->spinOnce(1);
		usleep(1000);
	}


	return 0;
}

