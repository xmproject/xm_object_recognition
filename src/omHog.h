//重构之后准备将统计向量 之间的角度作为聚类判断标准
#pragma once

#include <vector>
#include <omPix.h>

class HOG
{
public:
    HOG()
    {
       // std::cout<<"NEW HOG IS COMING"<<std::endl;
    }

public:
    std::vector<int> Eigen_vector;

public:
    void Clear();
    void AddPix(int r,int g,int b); 
    void CanculateEigenVector();
    double EigenDistance(HOG otherHOG);  

private:
    std::vector<PIX> Pix_vector;
    std::vector<int> COLOR_vector;
    int PointCount;
};
