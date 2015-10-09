#include "omFilter.h"
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGBA PointType;

 //直接通滤波器 通段‘z‘在1m以内的点云
pcl::PointCloud<PointType>::Ptr Filter::PassThrough(const pcl::PointCloud<PointType>::Ptr &inputcloud)
{
    pcl::PointCloud<PointType>::Ptr cloud_PassThought(new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(inputcloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,1.0);
    pass.filter(*cloud_PassThought);

    return cloud_PassThought;
}


//降采样滤波器 采样间隔0.002m
pcl::PointCloud<PointType>::Ptr  Filter::DeSamping(const pcl::PointCloud<PointType>::Ptr &inputcloud)
{
    pcl::VoxelGrid<PointType> vg;
    pcl::PointCloud<PointType>::Ptr cloud_DeSamping(new pcl::PointCloud<PointType>);
    vg.setInputCloud(inputcloud);
    vg.setLeafSize(0.0025f,0.0025f,0.0025f);
    vg.filter(*cloud_DeSamping);
    //std::cout <<cloud_DeSamping->points.size()<< "Points after DeSampling" << std::endl;
    return cloud_DeSamping;
}

//除去大面积平面点云 分割标准0.01m以内视为一个平面 该模块对于粗糙平面效果不好，有待改进
   pcl::PointCloud<PointType>::Ptr Filter::RemovePlane(const pcl::PointCloud<PointType>::Ptr &inputcloud)
   {
       pcl::SACSegmentation<PointType> seg;
       pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
       pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
       pcl::PointCloud<PointType>::Ptr cloud_RemovePlane (new pcl::PointCloud<PointType>());
       *cloud_RemovePlane=*inputcloud;
       pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
       seg.setOptimizeCoefficients(true);
       seg.setModelType(pcl::SACMODEL_PLANE);
       seg.setMethodType(pcl::SAC_RANSAC);
       seg.setMaxIterations(20);
       seg.setDistanceThreshold(0.01);

       int i=0,nr_points=(int) cloud_RemovePlane->points.size();
       while(cloud_RemovePlane->points.size()>0.3*nr_points)
       {
           seg.setInputCloud(cloud_RemovePlane);
           seg.segment(*inliers,*coefficients);
           if(inliers->indices.size()==0)
           {
               //std::cout<<"cloud not get a plaanar model for the given dataset."<<std::endl;
               break;
           }

           pcl::ExtractIndices<PointType> extract;
           extract.setInputCloud(cloud_RemovePlane);
           extract.setIndices(inliers);
           extract.setNegative(false);

           extract.filter(*cloud_plane);
           //std::cout<<"cloud representing the planar component:"<<cloud_plane->points.size()<<"points"<<std::endl;

           extract.setNegative(true);
           extract.filter(*cloud_RemovePlane);
       }
       //std::cout <<cloud_RemovePlane->points.size()<< "Points after Removeplane" << std::endl;
       return cloud_RemovePlane;
   }


   int Filter::ExtractionObject(const pcl::PointCloud<PointType>::Ptr &inputcloud, std::vector<pcl::PointIndices> &cluster_indices)
      {
          cluster_indices.clear();
          if (inputcloud->empty())
              return -1;
          pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
          tree->setInputCloud(inputcloud);


          pcl::EuclideanClusterExtraction<PointType> ec;
          ec.setClusterTolerance(0.02);
          ec.setMinClusterSize(200);
          ec.setMaxClusterSize(10000);
          ec.setSearchMethod(tree);
          ec.setInputCloud(inputcloud);
          ec.extract(cluster_indices);

          //std::cout<<"SEGED   "<<cluster_indices.size()<<"   cluster!!!"<<std::endl;
          return 0;
      }
