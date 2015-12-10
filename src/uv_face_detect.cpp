#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cvDia.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <uv_msgs/FacesDetected.h>
#include <uv_msgs/ImageBoundingBox.h>

#define default_ID "camera"

ros::Publisher _pub1,_pub2;
ros::Subscriber sub;

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

int peopleFound=0,nFaces,notValidtd;
int imgRedFactor = 2;
CvRect *bBfaces,*rectNoFaces;  
Mat image;
char haarCascade[]="/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

uv_msgs::FacesDetected faces;
uv_msgs::FacesDetected notValidFaces;
uv_msgs::ImageBoundingBox  bBoxfaces;

/**************************************************/

int fillFacesBoundingBoxes(CvRect rect)
{
  int x,y,w,h;
  x=rect.x*imgRedFactor;
  y=rect.y*imgRedFactor;
  w=rect.width*imgRedFactor;
  h=rect.height*imgRedFactor;
 
  bBoxfaces.center.u=x+w/2;
  bBoxfaces.center.v=y+h/2;
  bBoxfaces.width=w;
  bBoxfaces.height=h;
  bBoxfaces.cornerPoints[0].u=x;
  bBoxfaces.cornerPoints[0].v=y;
  bBoxfaces.cornerPoints[1].u=x+w;
  bBoxfaces.cornerPoints[1].v=y;
  bBoxfaces.cornerPoints[2].u=x+w;
  bBoxfaces.cornerPoints[2].v=y+h;
  bBoxfaces.cornerPoints[3].u=x;
  bBoxfaces.cornerPoints[3].v=y+h;

  return 0;
}


int getFacesData()
{
  int i;
  if (peopleFound>0){
    for(i=0;i<peopleFound;i++) {
      fillFacesBoundingBoxes(bBfaces[i]);
      faces.faces.push_back(bBoxfaces);
    }
  }

  if (notValidtd>0){
    for(i=0;i<notValidtd;i++) {
      fillFacesBoundingBoxes(rectNoFaces[i]);
      notValidFaces.faces.push_back(bBoxfaces);
    }
  }
  return 0;
}


void faceDetectCallback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage* iplImg;
  Mat tempImg;
  int width, height;

  cv_bridge::CvImageConstPtr cv_ptr;

  try { 
    cv_ptr = cv_bridge::toCvShare(msg,enc::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  width=cv_ptr->image.cols;
  height=cv_ptr->image.rows;

  iplImg=cvCreateImage(cvSize(width/2,height/2),IPL_DEPTH_8U,3);
  resize(cv_ptr->image,tempImg,cvSize(width/2,height/2));//,0,0,CV_INTER_LINEAR);  
  iplImg->imageData = (char *) tempImg.data;
  peopleFound=cvDiaFindFaces(iplImg,&nFaces,&bBfaces,&rectNoFaces);

  notValidtd=nFaces-peopleFound;
  faces.NoFaces=peopleFound;
  notValidFaces.NoFaces=notValidtd;

  double time = ros::Time::now().toSec();
  if (faces.NoFaces>0){
    faces.header.stamp=ros::Time(time);
    faces.header.frame_id=default_ID;
    getFacesData();
    _pub1.publish(faces);
    faces.faces.clear(); 
  }

  if (notValidFaces.NoFaces>0){
    notValidFaces.header.stamp=ros::Time(time);
    notValidFaces.header.frame_id=default_ID;
    getFacesData();
    _pub2.publish(notValidFaces);
    notValidFaces.faces.clear(); 
  }
}

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uv_face_detect");  
  ros::NodeHandle _nh;

  cvDiaInitPeopleDet(40/imgRedFactor,100/imgRedFactor,haarCascade);

  sub = _nh.subscribe("/camera/rgb/image_rect_color", 1, faceDetectCallback);
  _pub1=_nh.advertise<uv_msgs::FacesDetected>("/faceDetection/validFaces",100);
  _pub2=_nh.advertise<uv_msgs::FacesDetected>("/faceDetection/notValidFaces",100);

  while (ros::ok())
    { 
      ros::spinOnce();
    }

  return 0;
}
