/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 08.11.2011
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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <rosbag/bag.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#include <sensor_msgs/image_encodings.h>
#include <rosmrsmap/ObjectData.h>

#include <Eigen/Core>

#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <mrsmap/utilities/utilities.h>


using namespace mrsmap;

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




class CollectData
{
public:

	CollectData() {}
	~CollectData() {}

	void onInit( ros::NodeHandle& nh ) {

		sub_cloud_ = nh.subscribe( "input_cloud", 1, &CollectData::dataCallback, this );

		// parameters
		nh.param< std::string >( "data_folder", bag_folder_, "." );
		nh.param< std::string >( "data_file", bag_name_, "record.bag" );


		recording_ = false;
		frame_idx_ = 0;

		has_image_ = false;
	}


    void update() {

    	// while paused: display current image
    	// when running: display nothing to save some cycles
		if( has_image_ && !recording_ ) {

	    	boost::lock_guard<boost::mutex> lock(mutex);

			cv::Mat img_input, img_show;

			img_input = img_rgb.image;

			img_input.copyTo( img_show );

			while( img_show.cols > 640 ) {
				cv::Mat img_show2;
				cv::resize( img_show, img_show2, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
				img_show2.copyTo( img_show );
			}

			cv::Point2f p_center( img_show.cols/2, img_show.rows/2 );
			cv::Scalar c( 0.f, 255.f, 0.f );

			cv::Point2f p1 = p_center + cv::Point2f( -30, -40 );
			cv::Point2f p2 = p_center + cv::Point2f( -10, +40 );
			cv::rectangle( img_show, p1, p2, c, -1 );

			p1 = p_center + cv::Point2f( +10, -40 );
			p2 = p_center + cv::Point2f( +30, +40 );
			cv::rectangle( img_show, p1, p2, c, -1 );

			char str[255];
			sprintf(str, "%ix%i", img_input.cols, img_input.rows);
			cv::putText( img_show, std::string( str ), p_center + cv::Point2f( 0, -100 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, c );

			cv::imshow( "Collect Data", img_show );
		}

		// mouse input
		MouseEvent mouse;
		cv::setMouseCallback( "Collect Data", onMouse, &mouse );

		if( !recording_ )
			cv::waitKey(2);
		else
			cv::waitKey(1);


        if( mouse.event == CV_EVENT_MBUTTONDOWN ) {

		boost::lock_guard<boost::mutex> lock(mutex);

		ROS_INFO("writing test data to bag file..");

            rosbag::Bag bag;
            bag.open( bag_folder_ + "/" + bag_name_, rosbag::bagmode::Write );

            for( std::list< rosmrsmap::ObjectData >::iterator it = record_.begin(); it != record_.end(); it++ ) {

                ros::Time currTime = it->image_rgb.header.stamp;

				bag.write( "object_data", currTime, *it );

            }

            bag.close();

        	ROS_INFO("..done");
        }
        else if( mouse.event == CV_EVENT_LBUTTONDOWN ) {

	    	boost::lock_guard<boost::mutex> lock(mutex);

        	// toggle recording mode
        	recording_ = !recording_;

        }


    }


    void dataCallback( const sensor_msgs::PointCloud2ConstPtr& point_cloud ) {

    	boost::lock_guard<boost::mutex> lock(mutex);

    	ros::Time time = point_cloud->header.stamp;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );

		try {
			pcl::fromROSMsg(*point_cloud, *pointCloudIn);
		} catch (std::exception &e) {
			ROS_ERROR("Exception while converting point cloud");
			return;
		}


		rosmrsmap::ObjectData r;

		img_rgb.header = point_cloud->header;
		img_rgb.encoding = "bgr8";
		img_depth.header = point_cloud->header;
		img_depth.encoding = "32FC1";


		if( pointCloudIn->height == 1 ) {

			img_rgb.image = cv::Mat( 480, 640, CV_8UC3, 0.f );
			img_depth.image = cv::Mat( 480, 640, CV_32FC1, std::numeric_limits<float>::quiet_NaN() );
			reprojectPointCloudToImagesF( pointCloudIn, img_rgb.image, img_depth.image );

		}
		else {
			pointCloudToImages( pointCloudIn, img_rgb.image, img_depth.image );
		}


		has_image_ = true;

		if( recording_ && record_.size() < 1100 ) {

			img_rgb.toImageMsg( r.image_rgb );
			img_depth.toImageMsg( r.image_depth );

			record_.push_back( r );
			ROS_ERROR("recording frame %i", record_.size());

		}

    }


public:

	ros::Subscriber sub_cloud_;

	std::list< rosmrsmap::ObjectData > record_;

	sensor_msgs::PointCloud2 point_cloud_;

	cv_bridge::CvImage img_rgb, img_depth;

	std::string bag_name_, bag_folder_;

	bool recording_;
	int frame_idx_;
	bool has_image_;

	boost::mutex mutex;


};


int main(int argc, char** argv) {

	ros::init(argc, argv, "collect_data");
	ros::NodeHandle n("collect_data");
	CollectData cd;
	cd.onInit( n );

	ros::Rate loop_rate(100);
	while (n.ok())
	{
		ros::spinOnce();
		cd.update();
		loop_rate.sleep();
	}


}

