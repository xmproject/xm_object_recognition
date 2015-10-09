#include <iostream>
#include "omHog.h"
#include <pcl/common/common.h>
#include <pcl/console/print.h>

void HOG::Clear()
{
        Pix_vector.clear();
        PointCount=0;
        COLOR_vector.clear();
        Eigen_vector.clear();
}

void HOG::AddPix(int r ,int g,int b)
{
        PIX tempPix(r,g,b);
        Pix_vector.push_back(tempPix);
        PointCount++;
}

//现在有个新的思路就是把整个RGB作为点云的三个坐标来处理 问题在于再寻找K近临接的时候可能会找到多个相同的点，这个问题存在不可预知的规避性
void HOG::CanculateEigenVector()
{
       std::cout<<"Canculate Start"<<std::endl;
       pcl::console::print_info("We have %d point to canculate\n",Pix_vector.size());
       Eigen_vector.clear();
       Eigen_vector.resize(1000);
       int err1=0;
       int err2=0;
       for(int i=0;i<Pix_vector.size();i++)
       {
           float temp;
           temp=(float)Pix_vector[i].R/255.0*9.0*100.0+(float)Pix_vector[i].G/255.0*9.0*10.0+(float)Pix_vector[i].B/255.0*9.0;
           int num=temp;

           if(num<0)
           {
               //pcl::console::print_warn("Some points are not safety type1\n");
               err1++;
           }
           else if(num>999)
           {
               pcl::console::print_warn("The num is %d\n",num);
               //pcl::console::print_warn("Some points are not safety type2\n");
               err2++;
           }
           else
           {
           Eigen_vector[num]++;
           //std::cout<<(int) temp<<"    temp"<<std::endl;
           }
       }
       pcl::console::print_info("Total %d points are ill\n",err1+err2);
       std::cout<<"Canculate End"<<std::endl;
}

double HOG::EigenDistance(HOG otherHOG)
{
       // cout<<"Distance canculate start:  "<<this->Eigen_vector.size()<< "    "<<otherHOG.Eigen_vector.size()<<endl;
        double distance=0;
        double Eigen1_Lenth=0;
        double Eigen2_Lenth=0;
        double Eigen1XEigen2=0;
        for(int i=0;i<1000;i++)
        {
            Eigen1XEigen2+=(this->Eigen_vector[i]*otherHOG.Eigen_vector[i]);
            Eigen1_Lenth+=(this->Eigen_vector[i]*this->Eigen_vector[i]);
            Eigen2_Lenth+=(otherHOG.Eigen_vector[i]*otherHOG.Eigen_vector[i]);
        }
        Eigen1_Lenth=sqrt(Eigen1_Lenth);
        Eigen2_Lenth=sqrt(Eigen2_Lenth);

        distance=Eigen1_Lenth*Eigen2_Lenth;
        distance=Eigen1XEigen2/distance;
        return distance;
    }
