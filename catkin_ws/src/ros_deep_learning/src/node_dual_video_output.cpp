/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/videoOutput.h>

#include <iostream>
#include <fstream>

#include <thread>

// globals	
videoOutput* stream = NULL;
imageConverter* image_cvt = NULL;

std::string topic_name;

std::ofstream myfile1;

int count =0;


void task(const ImageConstPtr& input) {
    	// convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// render the image
	stream->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());

	
	//write timestamps to file
	ROS_INFO("%s -> %d", topic_name.c_str(), count);
	myfile << count << "-" << input->header.stamp.sec << input->header.stamp.nsec << "\n";
	count++;

	// update status bar
	char str[256];
	sprintf(str, "%s (%ux%u) | %.1f FPS", topic_name.c_str(), image_cvt->GetWidth(), image_cvt->GetHeight(), stream->GetFrameRate());
	stream->SetStatus(str);	

	// check for EOS
	if( !stream->IsStreaming() )
		ROS_SHUTDOWN();
}

int main (int argc, char ** argv) {
    using namespace boost;
    thread thread_1 = thread(task1);
    thread thread_2 = thread(task2);

    // do other stuff
    thread_2.join();
    thread_1.join();
    return 0;
}

// input image subscriber callback
void callback(const ImageConstPtr& input, const ImageConstPtr& input1)
//void img_callback( const sensor_msgs::ImageConstPtr input )
{

	thread thread_1 = thread(task1, );
    	thread thread_2 = thread(task2);
	// convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// render the image
	stream->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());

	
	//write timestamps to file
	ROS_INFO("%s -> %d", topic_name.c_str(), count);
	myfile << count << "-" << input->header.stamp.sec << input->header.stamp.nsec << "\n";
	count++;

	// update status bar
	char str[256];
	sprintf(str, "%s (%ux%u) | %.1f FPS", topic_name.c_str(), image_cvt->GetWidth(), image_cvt->GetHeight(), stream->GetFrameRate());
	stream->SetStatus(str);	

	// check for EOS
	if( !stream->IsStreaming() )
		ROS_SHUTDOWN();
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("video_output");

	/*
	 * declare parameters
	 */
	videoOptions video_options;

	

	std::string resource_str;
	std::string codec_str;
	std::string ts_fileName;	

	int video_bitrate = video_options.bitRate;

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("bitrate", video_bitrate);

	ROS_DECLARE_PARAMETER("ts_fileName", ts_fileName);
	
	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("bitrate", video_bitrate);

	ROS_GET_PARAMETER("ts_fileName", ts_fileName);

	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	video_options.bitRate = video_bitrate;

  	myfile.open(ts_fileName);

	ROS_INFO("opening video output: %s", resource_str.c_str());


	/*
	 * create stream
	 */
	stream = videoOutput::Create(resource_str.c_str(), video_options); 
	
	if( !stream )
	{
		ROS_ERROR("failed to open video output");
		return 0;
	}


	/*
	 * create image converter
	 */
	image_cvt = new imageConverter();

	if( !image_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * subscribe to image topic
	 */
//	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
//	auto img_sub1 = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in1", 5, img_callback);

        message_filters::Subscriber<Image> img_sub(nh, "image_in", 1);
        message_filters::Subscriber<Image> img_sub1(nh, "image_in1", 1);
        TimeSynchronizer<Image, Image> sync(img_sub, img_sub2, 10);
        sync.registerCallback(boost::bind(&callback, _1, _2));
	
	topic_name = ROS_SUBSCRIBER_TOPIC(img_sub);



	/*
	 * start streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	ROS_INFO("video_output node initialized, waiting for messages");
	ROS_SPIN();


	/*
	 * free resources
	 */
	myfile.close();
	
	delete stream;
	delete image_cvt;

	return 0;
}

