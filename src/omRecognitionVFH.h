#include <iostream>
#include <vector>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include "omModelVFH.h"

#pragma once

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef std::pair<std::string,int> name_model;

class RecognitionVFH
{
public:
    RecognitionVFH();
    bool ObjectReady();
    void Recognition_VFH_Prapare();
    void Recognition_VFH(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputcloud,std::vector<pcl::PointIndices> &cluster_indices);
    int Learing_VFH(pcl::PointCloud<PointType>::Ptr &inputcloud,std::string tempName,int j,std::vector<pcl::PointIndices> &cluster_indices);
    void Clean();
    ModelVFH GetObject();//abandon


private:
    void LoadFeatureModels_VFH(const boost::filesystem::path &base_dir, const std::string &extension, std::vector<ModelVFH> &modelsVFH);
    bool loadHist_VFH(const boost::filesystem::path &path, ModelVFH &vfh);
    void Recognition_VHF_RUN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputcloud,std::string name);
    int nearestKsearch_VFH(ModelVFH object);
    double CanculateDistance(ModelVFH model1,ModelVFH model2);

    std::vector<ModelVFH> modelsVFH;
    ModelVFH Object;
};
