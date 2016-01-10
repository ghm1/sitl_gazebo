/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_pixy_cam_plugin.h"

#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(PixyCameraPlugin)

/////////////////////////////////////////////////
PixyCameraPlugin::PixyCameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0),
  imgCounter(0)
{
    lastTimeImg = std::clock();
}

/////////////////////////////////////////////////
PixyCameraPlugin::~PixyCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void PixyCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "PixyCameraPlugin requires a CameraSensor.\n";
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
  }

  this->camera = this->parentSensor->GetCamera();

  if (!this->parentSensor)
  {
    gzerr << "PixyCameraPlugin not attached to a camera sensor\n";
    return;
  }

  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();


  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_pixy_cam_plugin] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);
 // opticalFlow_pub_ = node_handle_->Advertise<opticalFlow_msgs::msgs::opticalFlow>(topicName, 10);



  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      boost::bind(&PixyCameraPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void PixyCameraPlugin::OnNewFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format)
{
  _image = this->camera->GetImageData(0);
  //GetHFOV gives fucking gazebo::math::Angle which you can not cast...
  const double Hfov = 0.91;
  const double focal_length = (_width/2)/tan(Hfov/2);

  double rate = this->camera->GetRenderRate();
  if (!isfinite(rate)) {
	   rate =  10.0;
  }

  Mat frame = Mat(_height, _width, CV_8UC3);
  //Mat frameBGR = Mat(_height, _width, CV_8UC3);
  frame.data = (uchar*)_image; //frame is not the right color now -> convert

  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  //check time
  if( (std::clock() - lastTimeImg) / (double) CLOCKS_PER_SEC > 0.5 )
  {
      std::string dir = std::string("/home/michael/sourcecode/quadcopter/Firmware-ghm1/Tools/sitl_gazebo/images/");
      std::string origImgName = dir + std::string("orig") + to_string(imgCounter) + std::string(".png");
      std::string thresImgName = dir + std::string("thres") + to_string(imgCounter) + std::string(".png");
      std::string resultImgName = dir + std::string("result") + to_string(imgCounter) + std::string(".png");
      try {
          printf("saving img.. \n");
          //printf(imgName.c_str());
          //printf("\n");

          imwrite(origImgName, frame, compression_params);

          Mat channel[3];
          // The actual splitting.
          split(frame, channel);

          //thresholding
          cv::Mat imageThresh;
          cv::adaptiveThreshold(channel[0], imageThresh, 255,
                  cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 100, 0 );

          //save thesholded image
          imwrite(thresImgName, imageThresh, compression_params);

          //remove noise?

          //contourfinder
          std::vector<std::vector<cv::Point>> contours;
          std::vector<cv::Vec4i> hierarchy;
          cv::findContours(imageThresh, contours, hierarchy,
                           cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

          //find bounding rectangles of contours
          vector<vector<Point> > contours_poly( contours.size() );
          vector<Rect> boundRect( contours.size() );
          vector<Point2f>center( contours.size() );
          vector<float>radius( contours.size() );
          for( size_t i = 0; i < contours.size(); i++ )
          {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            minEnclosingCircle( contours_poly[i], center[i], radius[i] );
          }

          //draw rectangles into image
          RNG rng(12345);
          Mat drawing = Mat::zeros( imageThresh.size(), CV_8UC3 );
          for( size_t i = 0; i< contours.size(); i++ )
          {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
          }

          //save resulting image
          imwrite(resultImgName, drawing, compression_params);

          imgCounter++;
          //reset timer
          lastTimeImg = std::clock();
      }
      catch (runtime_error& ex) {
          fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
          return;
      }
  }


/*
  old_gray = frame_gray.clone();

  //std::cout << "pixel flow x = " << pixel_flow_x_integral << "   pixel flow y = " << pixel_flow_y_integral << endl;

  opticalFlow_message.set_time_usec(100000000000000);//big number to prevent timeout in inav
  opticalFlow_message.set_sensor_id(2.0);
  opticalFlow_message.set_integration_time_us(dt * 1000000);
  opticalFlow_message.set_integrated_x(flow_x_ang);
  opticalFlow_message.set_integrated_y(flow_y_ang);
  opticalFlow_message.set_integrated_xgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_ygyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_zgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_temperature(20.0);
  opticalFlow_message.set_quality(meancount * 255 / maxfeatures); //features?
  opticalFlow_message.set_time_delta_distance_us(0.0);
  opticalFlow_message.set_distance(0.0); //get real values in gazebo_mavlink_interface.cpp

  opticalFlow_pub_->Publish(opticalFlow_message); */

}
