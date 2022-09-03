#include <iostream>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <ctime>
#include <Eigen/Dense>

int iterm;
std::string map_surround_name;
std::vector<std::pair<std::string, std::vector<int>>> params;
const std::string unit_path = "../unit/pcd_nocolor/";   // remember to change
const std::string save_path = "../";

const float x_trans = 5;
const float y_trans = 5;
const float z_trans = 0;

const float r_delta = 30. * M_PI / 180;
const float p_delta = 0. * M_PI / 180;
const float y_delta = 0. * M_PI / 180;

const float kd_num = 300;

const float scale = 0.8;

class Map{
public:
    Map(){
        allocateMemory();
        std::cout << "********************************************" << std::endl;
        std::cout << "map generation initlization completed" << std::endl;
        std::cout << "********************************************" << std::endl;
        std::cout << "\n";

        std::cout << "param 1:" << "  " << "translate" << "  " << " x:" << "  " << x_trans << "    " << "y:" << "  " << y_trans << "   " << "z:" << "  " << z_trans << std::endl;
        std::cout << "param 2:" << "  " << "rotate" << "  " << " r:" << "  " << r_delta << "    " << "p:" << "  " << p_delta << "   " << "y:" << "  " << y_delta << std::endl;
        std::cout << "param 3:" << "  " << "destroy random" << "  " << " kd_num:" << "  " << kd_num << std::endl;
        std::cout << "param 4:" << "  " << "scaling" << "  " << " scale:" << "  " << scale << std::endl;
        std::cout << "\n";
    }

    // // the color api needs to be reconstructed
    // void readVtkCloud(){
    //     for(int i = 0; i < params.size(); i++){
    //         reader->SetFileName((unit_path + params[i].first).c_str());
    //         reader->Update();
    //         vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
    //         pcl::io::vtkPolyDataToPointCloud(polydata, *unitcloudrgb_vec[i]);
    //         std::cout << "unit" << "  " << i + 1  << "  " << "has" << "  " << unitcloudrgb_vec[i]->points.size() << "  " << "points" << std::endl;
    //      }
    // }

    void readPcdCloud(){
        pcl::io::loadPCDFile((unit_path + map_surround_name).c_str(), *map_surround);
        std::cout << "map surround has" << "  " << map_surround->points.size() << "  " << "points" << std::endl;
        std::cout << "\n";

        for(int i = 0; i < params.size(); i++){    
            pcl::io::loadPCDFile((unit_path + params[i].first).c_str(), *unitcloud_vec[i]);
            pcl::copyPointCloud(*unitcloud_vec[i], *unitcloud_vec_trans[i]);
            std::cout << "unit" << "  " << i + 1<< "  " << "has" << "  " << unitcloud_vec[i]->points.size() << "  " << "points" << std::endl;
        }
    }
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ> getModel2World(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cloud_){
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ tmp_center;
        pcl::copyPointCloud(*model_cloud_, *tmp_cloud);

        for(int i = 0; i < tmp_cloud->points.size(); i++){
            x_center  += tmp_cloud->points[i].x;
            y_center  += tmp_cloud->points[i].y;
            z_center  += tmp_cloud->points[i].z;
        }
        x_center /= tmp_cloud->points.size();
        y_center /= tmp_cloud->points.size();
        z_center /= tmp_cloud->points.size();
        tmp_center.x = x_center;
        tmp_center.y = y_center;
        tmp_center.z = z_center;

        for(int i = 0; i < tmp_cloud->points.size(); i++){
            tmp_cloud->points[i].x -= x_center;
            tmp_cloud->points[i].y -= y_center;
            tmp_cloud->points[i].z -= z_center;
        } 

        return std::make_pair(tmp_cloud, tmp_center);
    }

    // translate
    void getTransCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        for(int i = 0; i < tgt_cloud_->points.size(); i++){
            tgt_cloud_->points[i].x += x_trans;
            tgt_cloud_->points[i].y += y_trans;
            tgt_cloud_->points[i].z += z_trans;
        }
    }

    // rotate
    void getRotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        tmp_cloud = getModel2World(tgt_cloud_).first;
        pcl::PointXYZ tmp_center = getModel2World(tgt_cloud_).second;

        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(y_delta, Eigen::Vector3f(0, 0, 1)) *
               Eigen::AngleAxisf(p_delta, Eigen::Vector3f(0, 1, 0)) *
               Eigen::AngleAxisf(r_delta, Eigen::Vector3f(1, 0, 0));
        q.normalize();

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
        T.rotate(q.toRotationMatrix());  
        T.pretranslate(Eigen::Vector3f(tmp_center.x, tmp_center.y, tmp_center.z * std::cos(r_delta)));   // modify

        pcl::transformPointCloud(*tmp_cloud, *tgt_cloud_, T.matrix());
    }

    // destroy
    void getDestroyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        srand((unsigned int)time(0)); 
        int r = rand()%(tgt_cloud_->points.size());

        pcl::PointXYZ tmp_point = tgt_cloud_->points[r];
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(tgt_cloud_);
        std::vector<int> pointDestroyIdx(kd_num);
        std::vector<float> pointDestroyDis(kd_num);
        kdtree.nearestKSearch(tmp_point, kd_num, pointDestroyIdx, pointDestroyDis);

        // // no munus
        // *tgt_cloud_ = *src_cloud_ - *des_cloud;

        pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
        for(int i = 0; i < pointDestroyIdx.size(); i++){
            outliners->indices.push_back(pointDestroyIdx[i]);
        }
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud(tgt_cloud_);
          extract.setIndices(outliners);
          extract.setNegative(true);  
          extract.filter(*tgt_cloud_);
    }

    void getScaleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        tmp_cloud = getModel2World(tgt_cloud_).first;
        pcl::PointXYZ tmp_center = getModel2World(tgt_cloud_).second;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_i(new pcl::PointCloud<pcl::PointXYZ>());

        Eigen::Isometry3f s = Eigen::Isometry3f::Identity();
        s = Eigen::Scaling(scale, scale, scale);
        pcl::transformPointCloud(*tmp_cloud, *tmp_cloud_i, s.matrix());

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); 
        T.pretranslate(Eigen::Vector3f(tmp_center.x, tmp_center.y, tmp_center.z * scale));   // modify

        pcl::transformPointCloud(*tmp_cloud_i, *tgt_cloud_, T.matrix());
    }

    std::pair<float, Eigen::Matrix4f> getChange(const pcl::PointCloud<pcl::PointXYZ>& src_cloud_, const pcl::PointCloud<pcl::PointXYZ>& tgt_cloud_){
        
    }

    void getMap(){
        // for(int i = 0; i < unitcloudrgb_vec.size(); i++){
        //     *map_cloudrgb += *unitcloudrgb_vec[i];
        // }

        for(int i = 0; i < unitcloud_vec.size(); i++){
            *map_cloud += *unitcloud_vec[i];
        }
        
        for(int i = 0; i < unitcloud_vec_trans.size(); i++){
            *map_cloud_trans += *unitcloud_vec_trans[i];
        }
    }

    void addTrans2MapSurround(){
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        std::vector<int> samepoints_Idx;
        kdtree.setInputCloud(map_surround);
        for(int i = 0; i < unitcloud_vec.size(); i++){
            for(int j = 0; j < unitcloud_vec[i]->points.size(); j++){
                std::vector<int> pointSameIdx(1);
                std::vector<float> poinSameDis(1);
                kdtree.nearestKSearch(unitcloud_vec[i]->points[j], 1, pointSameIdx, poinSameDis);
                samepoints_Idx.push_back(pointSameIdx[0]);
            }    
        }
        pcl::PointIndices::Ptr sameIdx(new pcl::PointIndices());
        for(int i = 0; i < samepoints_Idx.size(); i++){
            sameIdx->indices.push_back(samepoints_Idx[i]);
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud(map_surround);
          extract.setIndices(sameIdx);
          extract.setNegative(true);  
          extract.filter(*map_surround_trans);

          for(int i = 0; i < unitcloud_vec_trans.size(); i++){
            *map_surround_trans += *unitcloud_vec_trans[i];
          }
    }

    void save(){
        std::cout << "\n";
        if(pcl::io::savePCDFile(save_path + "map_init.pcd", *map_cloud) != -1)
            std::cout << "map init has been saved" << std::endl;

        if(pcl::io::savePCDFile(save_path + "map_trans.pcd", *map_cloud_trans) != -1)
            std::cout << "map trans has been saved" << std::endl;
        
        if(pcl::io::savePCDFile(save_path + "map_surround.pcd", *map_surround) != -1)
            std::cout << "map surround has been saved" << std::endl;

        if(pcl::io::savePCDFile(save_path + "map_surround_trans.pcd", *map_surround_trans) != -1)
            std::cout << "map surround trans has been saved" << std::endl;
    }

    void allocateMemory(){
        // unitcloudrgb_vec.resize(iterm);
        // for(int i = 0; i < iterm; i++){
        //     unitcloudrgb_vec[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // }
        // map_cloudrgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // map_cloudrgb_trans.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        map_surround.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_surround_trans.reset(new pcl::PointCloud<pcl::PointXYZ>());

        unitcloud_vec.resize(iterm);
        for(int i = 0; i < iterm; i++){
            unitcloud_vec[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        }
        map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        

        unitcloud_vec_trans.resize(iterm);
        for(int i = 0; i < iterm; i++){
            unitcloud_vec_trans[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        }
        map_cloud_trans.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    // vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_surround;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_surround_trans;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> unitcloud_vec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> unitcloud_vec_trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_trans;

    // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> unitcloudrgb_vec;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloudrgb;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloudrgb_trans;
};


// main
int main(int argc, char **argv){
    if((argc - 2) % 5 != 0){
        std::cerr << "param error" << std::endl;
        return -1;
    }

    iterm = (argc - 2) / 5;
    
    std::cout << "\n";
    std::cout << "the map have" << "  " << iterm <<"  " <<  "units." << std::endl;
    std::cout << "\n";

    map_surround_name = (std::string) argv[1];  // map surround

    int order = 1;
    for(int i = 0; i < iterm; i++){
        std::vector<int> tmp_vec = {std::atoi(argv[order + 2]), std::atoi(argv[order + 3]), std::atoi(argv[order + 4]), std::atoi(argv[order + 5])};
        std::pair<std::string, std::vector<int>> tmp_pair = std::make_pair((std::string)argv[order + 1], tmp_vec);
        params.push_back(tmp_pair);
        order += 5;
    }

    Map map_generate;
    map_generate.readPcdCloud();
    
    for(int i = 0; i < iterm; i++){
        if(params[i].second[0] == 1) {map_generate.getTransCloud(map_generate.unitcloud_vec_trans[i]);}
        if(params[i].second[1] == 1) {map_generate.getRotateCloud(map_generate.unitcloud_vec_trans[i]);}
        if(params[i].second[2] == 1) {map_generate.getDestroyCloud(map_generate.unitcloud_vec_trans[i]);}
        if(params[i].second[3] == 1) {map_generate.getScaleCloud(map_generate.unitcloud_vec_trans[i]);}
    }
    map_generate.getMap();
    map_generate.addTrans2MapSurround();
    map_generate.save();

    return 0;
}
