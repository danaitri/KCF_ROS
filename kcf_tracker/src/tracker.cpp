#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.hpp"
#include <mva_msgs/mva_roi.h>
#include <tracker/controller.h>

#include <chrono>

int counter;

KCFTracker* kcfTracker = 0;
bool tracking_active = false;
ros::Publisher publisher;

bool init_tracker = false;
cv::Rect next_rect;

double rv = 0.0;

std::chrono::duration<double, std::ratio<1, 1000>> d1, d2;
std::chrono::time_point<std::chrono::high_resolution_clock> p1, p2;

mva_msgs::mva_roi ConvertToROI(cv::Rect cvrect)
{
	mva_msgs::mva_roi msgROI;
	msgROI.x = cvrect.x;
	msgROI.y = cvrect.y;
	msgROI.w = cvrect.width;
	msgROI.h = cvrect.height;
	msgROI.id = -1;
	return msgROI;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	  try
	  {
			if(init_tracker)
			{
				// convert to OpenCV
				cv::Mat img1 = cv_bridge::toCvShare(msg, "bgr8")->image;
		
				if( kcfTracker != 0 )
				{
					delete kcfTracker;
				}
				kcfTracker = new KCFTracker(true, true, true, true);
				kcfTracker->init(next_rect, img1);
				
				init_tracker = false;
				
				return;
			}

			if( kcfTracker != 0 )
			{


				p2 = std::chrono::high_resolution_clock::now();

				// convert to  OpenCV
				cv::Mat img2 = cv_bridge::toCvShare(msg, "bgr8")->image;				
				// tracking

				p1 = std::chrono::high_resolution_clock::now();
				cv::Rect roi = kcfTracker->update(img2);			
				d1 = std::chrono::high_resolution_clock::now() - p1;


				ROS_INFO("tracking lasted %f seconds", d1.count()/1000 );
				ROS_INFO("total loop in %f seconds", d2.count()/1000 );
				ROS_INFO("Frames per second:  %6.2f", (1000.0 / rv));
				
				
				// convert to mva_msgs::mva_roi

				mva_msgs::mva_roi msgROI = ConvertToROI(roi);

				// publish roi

				ROS_INFO("publishing tracker roi: [%d] [%d] [%d] [%d]", msgROI.x, msgROI.y, msgROI.w, msgROI.h);
				publisher.publish(msgROI);

				d2 = std::chrono::high_resolution_clock::now() - p2;
			        rv = 0.95 * rv + 0.05 * d2.count();		       			
			}
	
	  }

	  catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }


}

// Initialization service 

bool controlTracker(tracker::controller::Request& req, tracker::controller::Response & resp)
{	
		next_rect = cv::Rect(req.x, req.y, req.w, req.h);
		init_tracker = true;
		resp.result = 0;	
}



int main(int argc, char **argv)
{
	
	  counter = 0;
	  kcfTracker = 0;
	  ros::init(argc, argv, "kcf_tracker_node");

	  // part 1
	  // subscribe to usb_cam

	  ros::NodeHandle nh;
	  image_transport::ImageTransport it(nh);
	  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);


	  // part 2
	  // publish tracker roi

	  publisher = nh.advertise<mva_msgs::mva_roi>("tracker_roi", 100);

	  // part 3
	  // control tracker service

	  ros::ServiceServer service = nh.advertiseService("control_tracker", controlTracker);

	  ros::spin();

	  if(kcfTracker != 0)
		  delete kcfTracker;

	  service.shutdown();
}





