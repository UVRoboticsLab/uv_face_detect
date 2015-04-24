#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cvDia.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <GL/freeglut.h>
#include <uv_msgs/FacesDetected.h>
#include <uv_msgs/ImageBoundingBox.h>
#include <glip.h>

#define yMin -1.20
#define yMax 8.0
#define default_ID "camera"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
int peopleFound=0,nFaces,notValidtd;
int window;
int fbbData[128],noFacesData[128];
int imgRedFactor = 2;
glipImageSt *imageGlip;
glipDataSt *faceBBox,*noFaces;
CvRect *bBfaces,*rectNoFaces;  
IplImage* cvImage=NULL;	
Mat image;
char fileName[]="/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

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

int fillGlipData(int *data)
{
  data[0]= bBoxfaces.cornerPoints[0].u;     
  data[1]= bBoxfaces.cornerPoints[0].v;  
  data[2]= bBoxfaces.cornerPoints[1].u;     
  data[3]= bBoxfaces.cornerPoints[1].v; 
  data[4]= bBoxfaces.cornerPoints[1].u;     
  data[5]= bBoxfaces.cornerPoints[1].v; 
  data[6]= bBoxfaces.cornerPoints[2].u;     
  data[7]= bBoxfaces.cornerPoints[2].v; 
  
  data[8]= bBoxfaces.cornerPoints[2].u;     
  data[9]= bBoxfaces.cornerPoints[2].v; 
  data[10]= bBoxfaces.cornerPoints[3].u;     
  data[11]= bBoxfaces.cornerPoints[3].v; 
  data[12]= bBoxfaces.cornerPoints[3].u;     
  data[13]= bBoxfaces.cornerPoints[3].v; 
  data[14]= bBoxfaces.cornerPoints[0].u;     
  data[15]= bBoxfaces.cornerPoints[0].v; 
    
  return 0;
}



int getFacesData()
{
  int i;
  if (peopleFound>0){
    for(i=0;i<peopleFound;i++) {
      fillFacesBoundingBoxes(bBfaces[i]);
      fillGlipData(&fbbData[i*16]);
      faces.faces.push_back(bBoxfaces);
    }
  }
  if (notValidtd>0){
    for(i=0;i<notValidtd;i++) {
      fillFacesBoundingBoxes(rectNoFaces[i]);
      fillGlipData(&noFacesData[i*16]);
      notValidFaces.faces.push_back(bBoxfaces);
  }
  return 0;
}


void callback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage* iplImg;
  Mat tempImg;

  ros::NodeHandle _nh1,_nh2;
  ros::Publisher _pub1,_pub2;

  _pub1=_nh1.advertise<uv_msgs::FacesDetected>("/faceDetection/validFaces",100);
  //  cv_bridge::CvImagePtr cv_ptr;
  _pub2=_nh2.advertise<uv_msgs::FacesDetected>("/faceDetection/notValidFaces",100);

  cv_bridge::CvImagePtr cv_ptr;
  try { 
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //  image.imageData=(char*)cv_ptr->image.imageData;
  //  cvImage->imageData=(char*)cv_ptr->image.data;
  imageGlip->data=cv_ptr->image.data;

  iplImg=cvCreateImage(cvSize(imageGlip->width/2,imageGlip->height/2),IPL_DEPTH_8U,3);
  // cvtColor(cv_ptr->image,cv_ptr->image,CV_BGR2RGB);
  resize(cv_ptr->image,tempImg,cvSize(imageGlip->width/2,imageGlip->height/2));//,0,0,CV_INTER_LINEAR);
  
  iplImg->imageData = (char *) tempImg.data;
  peopleFound=cvDiaFindFaces(iplImg,&nFaces,&bBfaces,&rectNoFaces);

  notValidtd=nFaces-peopleFound;
  faceBBox->NoPts=peopleFound*8;
  noFaces->NoPts=notValidtd*8;

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
    _pub2.publish(notFaces);
    notValidFaces.faces.clear(); 
  }

  glipRedisplayImage(window);
  glutMainLoopEvent();

}

static void keyboard ( unsigned char key, int x, int y )
{
  switch ( key ) {
  case 27 :
  case 'q' :
  case 'Q' :
    exit(0);
    break;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uv_face_detect");  
  glutInitWindowSize(640,480);
  glutInit(&argc,argv);
  glutInitDisplayMode (GLUT_RGB);

  imageGlip=(glipImageSt*)glipCreateImage(640,480,1,GLIP_RGB,GL_UNSIGNED_BYTE);
  imageGlip->signY=-1;
  cvImage=cvCreateImageHeader(cvSize(640,480),IPL_DEPTH_8U,3);
  faceBBox=(glipDataSt*)glipCreateDataSt(0,2,2,GLIP_COLOR_GREEN,GLIP_LINES,GLIP_INT,fbbData);
  noFaces=(glipDataSt*)glipCreateDataSt(0,2,2,GLIP_COLOR_RED,GLIP_LINES,GLIP_INT,noFacesData);

  cvDiaInitPeopleDet(40/imgRedFactor,100/imgRedFactor,fileName);

  ros::NodeHandle nh_;
  ros::Subscriber sub;
  sub = nh_.subscribe("/camera/rgb/image_rect_color", 1, callback);
  window=glipDisplayImage(imageGlip,"Camera Image",0);
  glipDrawInImage(window,faceBBox);
  glipDrawInImage(window,noFaces);
  glutKeyboardFunc(keyboard);
 
  while (ros::ok())
    { 
      ros::spinOnce();
    }

  return 0;
}
