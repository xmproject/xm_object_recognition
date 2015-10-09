#include "omModelVFH.h"
typedef pcl::PointXYZRGBA PointType;

ModelVFH::ModelVFH():
clouds(new pcl::PointCloud<PointType>)
{
}

void ModelVFH::GetBoundingBox()
{
    if(clouds->size()==0)
    {
        printf("No clouds in \n");
        return ;
    }
    float x_min=999,y_min=999,z_min=999;
    float x_max=0,y_max=0,z_max=0;
    for(int i=0;i<clouds->size();i++)
        {
        //std::cout<<clouds->points[i].x<<"   "<<clouds->points[i].y<<"    "<<clouds->points[i].z<<std::endl;
            if(clouds->points[i].x<x_min)
                x_min=clouds->points[i].x;
            if(clouds->points[i].y<y_min)
                y_min=clouds->points[i].y;
            if(clouds->points[i].z<z_min)
                z_min=clouds->points[i].z;

            if(clouds->points[i].x>x_max)
                x_max=clouds->points[i].x;
            if(clouds->points[i].y>y_max)
                y_max=clouds->points[i].y;
            if(clouds->points[i].z>z_max)
                z_max=clouds->points[i].z;
        }
    bBox.x=x_min;
    bBox.y=y_min;
    bBox.z=z_min;

    bBox.height=y_max-y_min;
    bBox.width=x_max-x_min;
    bBox.depth=z_max-z_min;

    std::cout<<"x_min="<<x_min<<"   y_min="<<y_min<<"   z_min="<<z_min<<std::endl;
    std::cout<<"x_max="<<x_max<<"   y_max="<<y_max<<"   z_max="<<z_max<<std::endl;
    std::cout<<"width="<<bBox.width<<"   height="<<bBox.height<<"   depth="<<bBox.depth<<std::endl;
}


void ModelVFH::SetReady(bool flag)
{
    DataReady=flag;
}

bool ModelVFH::GetReady()
{
    return DataReady;
}
