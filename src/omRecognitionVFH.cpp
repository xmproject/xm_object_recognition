#include "omRecognitionVFH.h"
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include "omModelVFH.h"

#define THRESH 1000

RecognitionVFH::RecognitionVFH()
{
    Object.SetReady(false);
}

//************************************
//prepare
//************************************
void RecognitionVFH::Recognition_VFH_Prapare()
{
        pcl::console::print_info("VFH Prepare to recognition\n");
        std::string filepath="/home/cuiyufang/objectmodels";  //determine the corret file path of the trainging datas
        std::string extension(".pcd");
        //transform(extension.begin(),extension.end(),extension.begin(),(int(*)(int))towlower);//problem

        //load model histogram
        LoadFeatureModels_VFH(filepath,extension,modelsVFH);
        pcl::console::print_highlight("Loaded %d VFH models.\n",
                                      (int)modelsVFH.size ());

        //*********************************************************
        /*
        //convert to FLANN format
        flann::Matrix<float> data_temp(new float[models.size()*models[0].second.size()],models.size(),models[0].second.size());
        data=data_temp;
        pcl::console::print_highlight("the matrix has %d ---- %d size",
                                      (int)data.rows,(int)data.cols);

        for(size_t i=0;i<data.rows;++i)
            for(size_t j=0;j<data.cols;++j)
                data[i][j]=models[i].second[j];

        delete data_temp.ptr();
        */
}

void RecognitionVFH::LoadFeatureModels_VFH(const boost::filesystem::path &base_dir, const std::string &extension, std::vector<ModelVFH> &models)
    {
        if(!boost::filesystem::exists(base_dir)&&!boost::filesystem::is_directory (base_dir))
            return;
        for(boost::filesystem::directory_iterator it(base_dir);it!=boost::filesystem::directory_iterator();++it)
        {
            if (boost::filesystem::is_directory (it->status ()))
                {
                  std::stringstream ss;
                  ss << it->path ();
                  pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
                  LoadFeatureModels_VFH(it->path (), extension, models);
                }
            if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
            {
              ModelVFH m;
              if (loadHist_VFH (base_dir / it->path ().filename (), m))
                  models.push_back (m);
              else
                  std::cout<<"But We Loaded it failed"<<std::endl;
            }
        }
    }

bool RecognitionVFH::loadHist_VFH(const boost::filesystem::path &path, ModelVFH &vfh)
    {
        std::cout<<"Loading"<<path.string()<<" now"<<std::endl;
        //*********
        try
        {
            pcl::PointCloud<PointType>::Ptr read_pointcloud(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile(path.string(),*read_pointcloud);
            //pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);

            pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>() );
            ne.setInputCloud(read_pointcloud);
            ne.setRadiusSearch(0.03);
            ne.setSearchMethod(tree);
            ne.compute(*point_normal);

            /*
            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
            ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(0.02f);
            ne.setNormalSmoothingSize(10.0f);
            ne.setInputCloud(read_pointcloud);
            ne.compute(*point_normal);
            */

            pcl::VFHEstimation<pcl::PointXYZRGBA,pcl::Normal,pcl::VFHSignature308> vfh_Es;
            vfh_Es.setInputCloud(read_pointcloud);
            vfh_Es.setInputNormals(point_normal);

            //pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
            vfh_Es.setSearchMethod(tree);
            pcl::PointCloud<pcl::VFHSignature308>::Ptr point_VFH(new pcl::PointCloud<pcl::VFHSignature308>());
            vfh_Es.compute(*point_VFH);

            vfh.FeatureVFH.resize(308);

            for (size_t i = 0; i < 308; ++i)
            {
                vfh.FeatureVFH[i] = point_VFH->points[0].histogram[i];
            }

            std::string temp=path.string();
            std::size_t positionFirst=temp.find_first_of("Obj_")+4;
            std::size_t positionLast=temp.rfind("_Obj");

            temp.assign(temp,positionFirst,(positionLast-positionFirst));

            vfh.ObjectName=temp;
        }
        catch(pcl::InvalidConversionException e)
        {
            std::cout<<"Faild to load "<<path.string()<<std::endl;
            return (false);
        }
        std::cout<<"Loading"<<path.string()<<" success"<<std::endl;
        return (true);
        //*********
    }

//************************************
//learing
//************************************
int RecognitionVFH::Learing_VFH(pcl::PointCloud<PointType>::Ptr &inputcloud,std::string name,int j,std::vector<pcl::PointIndices> &cluster_indices)
   {
       if(cluster_indices.size()==1)
       {
         pcl::PCDWriter writer;
         std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();

         pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
         cloud_cluster->points.push_back (inputcloud->points[*pit]);
         cloud_cluster->width = cloud_cluster->points.size ();
         cloud_cluster->height = 1;
         cloud_cluster->is_dense = true;

         std::stringstream ss;
         ss << "/home/cuiyufang/objectmodels/"<<"Obj_"<<name <<"_Obj"<< j << ".pcd";
         writer.write<PointType> (ss.str (), *cloud_cluster, false);
         pcl::console::print_highlight("Saved a file\n");
       }
       else
       if(cluster_indices.size()==0)
           pcl::console::print_error("No cluster extraced\n");
       else
            pcl::console::print_error("Too Many cluster,%d cluster is here\n",cluster_indices.size());
       return 0;
   }


//************************************
//Recognition
//************************************
void RecognitionVFH::Recognition_VFH(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputcloud,std::vector<pcl::PointIndices> &cluster_indices)
    {
        Object.SetReady(false);
        if(cluster_indices.size()==0)
            return ;
        else
        {
            int num=1;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (inputcloud->points[*pit]);

              std::string tempName="Object"+num;
              Recognition_VHF_RUN(cloud_cluster,tempName);
              num++;
            }
        }
    }

void RecognitionVFH::Recognition_VHF_RUN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputcloud,std::string tempName)
   {
       ModelVFH object_histogram;
       pcl::PointCloud<pcl::Normal>::Ptr object_normal(new pcl::PointCloud<pcl::Normal>);

       pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> object_ne;
       pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr object_tree (new pcl::search::KdTree<pcl::PointXYZRGBA>() );
       object_ne.setInputCloud(inputcloud);
       object_ne.setRadiusSearch(0.03);
       object_ne.setSearchMethod(object_tree);
       object_ne.compute(*object_normal);

       pcl::VFHEstimation<pcl::PointXYZRGBA,pcl::Normal,pcl::VFHSignature308> object_vfh;
       object_vfh.setInputCloud(inputcloud);
       object_vfh.setInputNormals(object_normal);
       object_vfh.setSearchMethod(object_tree);
       pcl::PointCloud<pcl::VFHSignature308>::Ptr object_VFH(new pcl::PointCloud<pcl::VFHSignature308>());
       object_vfh.compute(*object_VFH);

       object_histogram.FeatureVFH.resize(308);

       for (size_t i = 0; i < 308; ++i)
       {
           object_histogram.FeatureVFH[i] = object_VFH->points[0].histogram[i];
       }

       object_histogram.ObjectName=tempName;

       int objectNum=nearestKsearch_VFH(object_histogram);

       if(objectNum==-1)
       {
           pcl::console::print_warn("No match .\n");
           Object.SetReady(false);
       }
       else
       {
           pcl::console::print_info("The nearest is %s\n",modelsVFH[objectNum].ObjectName.c_str());
           Object.ObjectName=modelsVFH[objectNum].ObjectName;
           *Object.clouds=*inputcloud;
           //std::cout<<"TEST"<<Object.clouds->size()<<std::endl;
           Object.SetReady(true);
       }
   }

int RecognitionVFH::nearestKsearch_VFH(ModelVFH object)
    {
        double nearest_dis=THRESH;
        int objectNum=-1;
        for(int i=0;i<modelsVFH.size();i++)
        {
            double dis=CanculateDistance(modelsVFH[i],object);
            if(dis<nearest_dis)
            {
                nearest_dis=dis;
                objectNum=i;
            }
        }
        pcl::console::print_warn("Dis is %f",nearest_dis);
        return objectNum;
    }

double RecognitionVFH::CanculateDistance(ModelVFH model1,ModelVFH model2)
   {
       if(model1.FeatureVFH.size()!=model2.FeatureVFH.size())
       {
           pcl::console::print_warn("Size is not good\n");
           return 0;
       }

       double dis=0;
       for(int i=0;i<model1.FeatureVFH.size();i++)
       {
           dis+=((model1.FeatureVFH[i]-model2.FeatureVFH[i])*(model1.FeatureVFH[i]-model2.FeatureVFH[i]));
       }
       printf("%s distance %f\n",model1.ObjectName.c_str(),dis);
       return dis;
   }

void RecognitionVFH::Clean()
{
    modelsVFH.clear();
    Object.SetReady(false);
}

ModelVFH RecognitionVFH::GetObject()
{
    if(Object.clouds->size()==0 || !Object.GetReady())
    {
        Object.SetReady(false);
    }
    else
    {
       Object.GetBoundingBox();
       Object.SetReady(true);
    }
    return Object;
}

bool RecognitionVFH::ObjectReady()
{
    return Object.GetReady();
}
