#include <pcl/common/common.h>
#include <string>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>

#pragma once

typedef pcl::PointXYZRGBA PointType;

class BoundingBox
{
public:
    float x;
    float y;
    float z;
    float width;
    float height;
    float depth;
};


class ModelVFH
{
public:
    ModelVFH();
    void GetBoundingBox();
    void SetReady(bool flag);
    bool GetReady();
    //void Get2DBoundingBox(cv::Rect& rect);

    std::string ObjectName;
    BoundingBox bBox;
    pcl::PointCloud<PointType>::Ptr  clouds;
    std::vector<float> FeatureVFH;
    bool DataReady;
    //cv::Rect 2Drect;
private:
    
};
