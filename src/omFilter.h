#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#pragma once

typedef pcl::PointXYZRGBA PointType;

class Filter
{
public:
    pcl::PointCloud<PointType>::Ptr PassThrough(const pcl::PointCloud<PointType>::Ptr &inputcloud);
    pcl::PointCloud<PointType>::Ptr  DeSamping(const pcl::PointCloud<PointType>::Ptr &inputcloud);
    pcl::PointCloud<PointType>::Ptr  RemovePlane(const pcl::PointCloud<PointType>::Ptr &inputcloud);
    int ExtractionObject(const pcl::PointCloud<PointType>::Ptr &inputcloud, std::vector<pcl::PointIndices> &cluster_indices);
};
