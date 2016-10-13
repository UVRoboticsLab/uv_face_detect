#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cvUV.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <uv_msgs/ImageBoundingBox.h>
#include <uv_msgs/ImageBoundingBoxListStamped.h>

#define default_ID "camera"
#define default_topic "/camera/rgb/image_rect_color"

ros::Publisher _pub1,_pub2;
ros::Subscriber sub;
image_transport::Publisher pub;

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

bool exportFaceImages=true;
int peopleFound=0,nFaces,notValidtd;
int imgRedFactor = 2;
CvRect *bBfaces,*rectNoFaces;  
Mat image,face;
char haarCascade[]="/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

uv_msgs::ImageBoundingBoxListStamped faces;
uv_msgs::ImageBoundingBoxListStamped notValidFaces;
uv_msgs::ImageBoundingBox  bBoxfaces;
sensor_msgs::ImagePtr faceImage; 

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
      faces.boxes.push_back(bBoxfaces);
    }
  }

  if (notValidtd>0){
    for(i=0;i<notValidtd;i++) {
      fillFacesBoundingBoxes(rectNoFaces[i]);
      notValidFaces.boxes.push_back(bBoxfaces);
    }
  }
  return 0;
}


void faceDetectCallback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage* iplImg;
  Mat tempImg,face;
  int width, height;

  cv_bridge::CvImageConstPtr cv_ptr;
  Rect faceRect;


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
  peopleFound=cvUVFindFaces(iplImg,&nFaces,&bBfaces,&rectNoFaces);

  notValidtd=nFaces-peopleFound;
  faces.NoBoxes=peopleFound;
  notValidFaces.NoBoxes=notValidtd;

  double time = ros::Time::now().toSec();
  if (faces.NoBoxes>0){
    faces.header.stamp=ros::Time(time);
    faces.header.frame_id=default_ID;
    getFacesData();
    _pub1.publish(faces);
    faces.boxes.clear(); 
    faceRect=Rect(bBfaces[0].x*2,bBfaces[0].y*2,bBfaces[0].width*2,bBfaces[0].height*2);
    //   Size tempSize=cvSize(10.0,10.0);
    //  bBfaces[0]+=tempSize;
    //    faceRect+=tempSize;
    face=cv::Mat(cv_ptr->image,faceRect).clone();
    faceImage = cv_bridge::CvImage(std_msgs::Header(), "rgb8", face).toImageMsg();
    pub.publish(faceImage);

  }

  if (notValidFaces.NoBoxes>0){
    notValidFaces.header.stamp=ros::Time(time);
    notValidFaces.header.frame_id=default_ID;
    getFacesData();
    _pub2.publish(notValidFaces);
    notValidFaces.boxes.clear(); 
  }
}

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uv_face_detect");  
  ros::NodeHandle _nh;
  string topic;

  _nh.getParam("uv_face_detect/image",topic);
  if (topic.size()==0) topic=default_topic;

  cvUVInitPeopleDet(40/imgRedFactor,100/imgRedFactor,haarCascade);
  
  sub = _nh.subscribe(topic, 1, faceDetectCallback);

  _pub1=_nh.advertise<uv_msgs::ImageBoundingBoxListStamped>("/faceDetection/validFaces",100);
  _pub2=_nh.advertise<uv_msgs::ImageBoundingBoxListStamped>("/faceDetection/notValidFaces",100);
  
  image_transport::ImageTransport it(_nh);
  pub = it.advertise("faceDetection/faceImg", 1);
 
  while (ros::ok())
    { 
      ros::spinOnce();
    }

  return 0;
}
