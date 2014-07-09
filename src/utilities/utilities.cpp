/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute VI, University of Bonn
 *  Author: Joerg Stueckler, 09.07.2014
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

#include "rosmrsmap/utilities/utilities.h"


void mrsmap::imagesToPointCloud( const sensor_msgs::Image& depthImg, const sensor_msgs::Image colorImg, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud ) {

	cloud->header = pcl_conversions::toPCL( depthImg.header );
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


