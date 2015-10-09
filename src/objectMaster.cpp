#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
//OpenCV specific includes
#include <opencv2/opencv.hpp>
//get clouds and images
#include "cloudTraver.h"

#include "omFilter.h"
#include "omRecognitionVFH.h"
#include "omModelVFH.h"

#include <iostream>
#include <string>

enum Process_State {Pro_LRARING,Pro_RECONGNIZE,
                    //Pro_LEARING_HOG,Pro_RECONGNIZE_HOG,Pro_RECONGNIZE_HOG_PREPARE,
                    Pro_LEARING_VFH,Pro_RECONGNIZE_VFH,Pro_RECONGNIZE_VFH_PREPARE,
                   Pro_READONLY,Pro_Error,Pro_Wait};
Process_State State;

//let cloud visible
pcl::visualization::CloudViewer viewer("X");
//pcl::visualization::CloudViewer viewerS("S");

void drawPoint(cv::Point2d p2d,cv::Mat& image,cv::Scalar s)
{
    cv::Point center( cvRound(p2d.x ), cvRound(p2d.y));
    cv::circle(image,center,5,s,1);
    std::cout<<p2d.x<<"   "<<p2d.y<<std::endl;
}

int main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "objectMaster");
     ros::NodeHandle n;
     ros::Rate waiting_rate(30);

     //inital for recognition
     RecognitionVFH recognitionvfh;
     ModelVFH modelvfh;

     //strat a traver and wait for its ready
     cloudTraver ct(n);
     while(!ct.isReady())
     {
         ros::spinOnce();
         waiting_rate.sleep();
     }

     State=Pro_RECONGNIZE_VFH_PREPARE;

     cvNamedWindow("CurrentImage",CV_WINDOW_AUTOSIZE);
     cv::Mat image;
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudP;


     std::string objectNameTemp="TEMP";
     int count=0;
    while(ros::ok())
    {
        while(!ct.isReady())
        {
          ros::spinOnce();
        }

        image=ct.getImage();
        cloudP=ct.getCloud();

        if(cloudP->empty())
        {
            std::cout<<"No pointCloud passed into this process, fuck you no points you play MAOXIAN!"<<std::endl;
            continue;
        }
        pcl::PointCloud<PointType>::Ptr cloud_RGBA(new pcl::PointCloud<PointType>);
        *cloud_RGBA=*cloudP;

        Filter filter;
        cloud_RGBA=filter.PassThrough(cloud_RGBA);
        cloud_RGBA=filter.DeSamping(cloud_RGBA);
        cloud_RGBA=filter.RemovePlane(cloud_RGBA);
        if(cloud_RGBA->empty())
        {
            std::cout<<"No pointCloud left after the samping"<<std::endl;
            continue;
        }

        std::vector<pcl::PointIndices> cluster_indices;
        filter.ExtractionObject(cloud_RGBA,cluster_indices);

        if(cluster_indices.size()!=0)
        {
            //std::cout<<cluster_indices.size()<<"clusters extraced"<<std::endl;
            switch(State)
            {
                case Pro_READONLY:
                    break;
                case Pro_LEARING_VFH:
                    recognitionvfh.Clean();
                    count++;
                    recognitionvfh.Learing_VFH(cloud_RGBA,objectNameTemp,count,cluster_indices);
                    recognitionvfh.Recognition_VFH_Prapare();
                    State=Pro_READONLY;
                    break;
                case Pro_RECONGNIZE_VFH:
                    modelvfh.SetReady(false);
                    recognitionvfh.Recognition_VFH(cloud_RGBA,cluster_indices);

                    if(recognitionvfh.ObjectReady())
                    {
                        modelvfh=recognitionvfh.GetObject();
                        std::cout<<modelvfh.ObjectName<<"      detected"<<std::endl;
                        //pcl::console::print_error("%s detected\n",modelvfh.ObjectName);

                        //////////////////////////
                        cv::Point2d p2d;
                        cv::Point3d p3d;
                        modelvfh.GetBoundingBox();
                    //1 white
                        p3d.x=modelvfh.bBox.x;
                        p3d.y=modelvfh.bBox.y;
                        p3d.z=modelvfh.bBox.z;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(255,255,255));
                    //2 yellow
                        p3d.x=modelvfh.bBox.x+modelvfh.bBox.width;
                        p3d.y=modelvfh.bBox.y;
                        p3d.z=modelvfh.bBox.z;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(0,255,255));
                    //3 magenta
                        p3d.x=modelvfh.bBox.x+modelvfh.bBox.width;
                        p3d.y=modelvfh.bBox.y;
                        p3d.z=modelvfh.bBox.z+modelvfh.bBox.depth;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(255,0,255));
                    //4 cyan
                        p3d.x=modelvfh.bBox.x;
                        p3d.y=modelvfh.bBox.y;
                        p3d.z=modelvfh.bBox.z+modelvfh.bBox.depth;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(255,255,0));
                    //5 blue
                        p3d.x=modelvfh.bBox.x;
                        p3d.y=modelvfh.bBox.y+modelvfh.bBox.height;
                        p3d.z=modelvfh.bBox.z;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(0,0,255));
                    //6 green
                        p3d.x=modelvfh.bBox.x+modelvfh.bBox.width;
                        p3d.y=modelvfh.bBox.y+modelvfh.bBox.height;
                        p3d.z=modelvfh.bBox.z;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(0,255,0));
                    //7 red
                        p3d.x=modelvfh.bBox.x+modelvfh.bBox.width;
                        p3d.y=modelvfh.bBox.y+modelvfh.bBox.height;
                        p3d.z=modelvfh.bBox.z+modelvfh.bBox.depth;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(255,0,0));
                    //8 black
                        p3d.x=modelvfh.bBox.x;
                        p3d.y=modelvfh.bBox.y+modelvfh.bBox.height;
                        p3d.z=modelvfh.bBox.z+modelvfh.bBox.depth;
                        ct.Point3Dto2D(p3d,p2d);
                        drawPoint(p2d,image,cv::Scalar(0,0,0));

                    }
                    else
                    {
                        //std::cout<<"No Sir"<<std::endl;
                    }
                    //State=Pro_READONLY;
                    break;
                case Pro_RECONGNIZE_VFH_PREPARE:
                    recognitionvfh.Recognition_VFH_Prapare();
                    State=Pro_READONLY;
                    break;
            }
        }

        viewer.showCloud(cloud_RGBA);
         //viewer.showCloud(cloudP);
        cv::imshow("CurrentImage", image);
        //U must wait a second to let the image show on the screen
        char temp=cvWaitKey(33);
        switch(temp)
        {
        case 'q':
            return 0;
        case 's':
            State=Pro_LEARING_VFH;
            cin>>objectNameTemp;
            break;
        case 'd':
            State=Pro_RECONGNIZE_VFH;
            break;
        case 'r':
            State=Pro_READONLY;
            break;
        }
    //wait for new datas
    }
}
