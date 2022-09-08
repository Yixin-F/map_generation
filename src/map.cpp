#include <iostream>
#include <ctime>
#include <algorithm>
#include <cmath>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/common/eigen.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

int iterm;
std::string map_surround_name;
std::vector<std::pair<std::string, std::vector<int>>> params;
const std::string unit_path = "../unit/pcd_nocolor/";   // remember to change
const std::string save_path = "../";

const float x_trans = 5;
const float y_trans = 5;
const float z_trans = 0;

const float r_delta = 15. * M_PI / 180;
const float p_delta = 0. * M_PI / 180;
const float y_delta = 0. * M_PI / 180;

const float kd_num = 300;
const float kd_unit = 3;  // need to check

const int kd_norm = 10;
const int corr_num = 100;

const float scale = 0.8;

const float icp_fitness = 0.2;

bool order(const std::pair<std::pair<int, int>, Eigen::Matrix4f>& a, const std::pair<std::pair<int, int>, Eigen::Matrix4f>& b){
        return (std::sqrt(std::pow(a.second(0, 3), 2) + std::pow(a.second(1, 3), 2) + std::pow(a.second(2, 3), 2)) < std::sqrt(std::pow(b.second(0, 3), 2) + std::pow(b.second(1, 3), 2) + std::pow(b.second(2, 3), 2)));
}

bool order1(const std::pair<int, std::pair<int, float>>& a, const std::pair<int, std::pair<int, float>>& b){
    return a.second.second > b.second.second;
}

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

    void readPcdCloud(){
        pcl::io::loadPCDFile((unit_path + map_surround_name).c_str(), *map_surround);
        std::cout << "map surround has" << "  " << map_surround->points.size() << "  " << "points" << std::endl;
        std::cout << "\n";

        for(int i = 0; i < params.size(); i++){    
            pcl::io::loadPCDFile((unit_path + params[i].first).c_str(), *unitcloud_vec[i]);
            pcl::copyPointCloud(*unitcloud_vec[i], *unitcloud_vec_trans[i]);
            std::cout << "unit" << "  " << i + 1<< "  " << "has" << "  " << unitcloud_vec[i]->points.size() << "  " << "points" << std::endl;
        }
        std::cout << "\n";
    }
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ> getModel2World(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cloud_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ tmp_center;
        pcl::copyPointCloud(*model_cloud_, *tmp_cloud);

        tmp_center = getCenterFromCloud(tmp_cloud);

        for(int i = 0; i < tmp_cloud->points.size(); i++){
            tmp_cloud->points[i].x -= tmp_center.x;
            tmp_cloud->points[i].y -= tmp_center.y;
            tmp_cloud->points[i].z -= tmp_center.z;
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


    bool check(const std::pair<std::pair<int, int>, Eigen::Matrix4f>& a, const std::vector<std::pair<std::pair<int, int>, Eigen::Matrix4f>>& match){
        for(int i = 0; i < match.size(); i++){
            if(a.first.first == match[i].first.first){
                return false;
            }
        }
        return true;  // right?
    }

    pcl::PointXYZ getCenterFromCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cloud_){
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;

        for(int i = 0; i < model_cloud_->points.size(); i++){
            x_center  += model_cloud_->points[i].x;
            y_center  += model_cloud_->points[i].y;
            z_center  += model_cloud_->points[i].z;
        }
        x_center /= model_cloud_->points.size();
        y_center /= model_cloud_->points.size();
        z_center /= model_cloud_->points.size();

        pcl::PointXYZ tmp_center;
        tmp_center.x = x_center;
        tmp_center.y = y_center;
        tmp_center.z = z_center;

        return tmp_center;
    }

    void getCenterMap(){
        for(int i = 0; i < unitcloud_vec.size(); i++){
            pcl::PointXYZ tmp_center;
            tmp_center = getCenterFromCloud(unitcloud_vec[i]);
            unitCenter_map->points.push_back(tmp_center);
        }       
    }

    std::vector<std::pair<int, float>> searchUnitByCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& search_cloud_){
        pcl::PointXYZ tmp_center;
        tmp_center = getCenterFromCloud(search_cloud_);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(unitCenter_map);
        std::vector<int> searchIdx(kd_unit);
        std::vector<float> searchDis(kd_unit);
        kdtree.nearestKSearch(tmp_center, kd_unit, searchIdx, searchDis);

        for(int i = 0; i < searchIdx.size(); i++){
            std::cout << "coarse search by unit ceter: " << " " << (params[searchIdx[i]].first).c_str() << " " << " dis is " << searchDis[i] << std::endl;
        }
        std::cout << "\n";

         std::vector<std::pair<int, float>> tmp_match;
        for(int i = 0; i < searchIdx.size(); i++){
            tmp_match.push_back(std::make_pair(searchIdx[i], searchDis[i]));
        }

        return tmp_match;
    }

    /***********************************fpfh + ransac + svd    some problem needs to be reconstructed!! *******************************************/
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree){
    //     /****  get fpfh ****/
    //     // normal estimate
    //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    //     pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    //     n.setInputCloud(cloud_);
    //     n.setNumberOfThreads(6);  // core 6
    //     n.setSearchMethod(tree);
    //     n.setKSearch(kd_norm);
    //     n.compute(*normals);

    //     // fpfh estimate
    //     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    //     pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    //     f.setNumberOfThreads(8); 
    //     f.setInputCloud(cloud_);
    //     f.setInputNormals(normals);
    //     f.setSearchMethod(tree);
    //     f.setKSearch(kd_norm);
    //     f.compute(*fpfh);

    //     return fpfh;
    // }

    // Eigen::Matrix4f getRegisterMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
    //     pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    //     pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    //     src_fpfh = getFPFH(src_cloud_, tree);
    //     tgt_fpfh = getFPFH(tgt_cloud_, tree);

    //     // get correspondence
    //     pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> src_tgt_corr;
    //     boost::shared_ptr<pcl::Correspondences> corrs(new pcl::Correspondences);
    //     src_tgt_corr.setInputSource(src_fpfh);
    //     src_tgt_corr.setInputTarget(tgt_fpfh);
    //     src_tgt_corr.determineReciprocalCorrespondences(*corrs);

    //     // // ransac
    //     boost::shared_ptr<pcl::Correspondences> corrs_inliers(new pcl::Correspondences);
    //     pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac;
    //     ransac.setInputSource(src_cloud_);
    //     ransac.setInputTarget(tgt_cloud_);
    //     ransac.setMaximumIterations(100);
    //     ransac.setInlierThreshold(0.1f);
    //     ransac.getRemainingCorrespondences(*corrs, *corrs_inliers);

    //     // // coarse register
    //     // pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> ransac;
    //     // ransac.setInputSource(src_cloud_);
    //     // ransac.setInputTarget(tgt_cloud_);
    //     // ransac.setSourceFeatures(src_fpfh);
    //     // ransac.setTargetFeatures(tgt_fpfh);
    //     // ransac.setCorrespondenceRandomness(5);
    //     // ransac.setInlierFraction(0.5f);
    //     // ransac.setNumberOfSamples(3);
    //     // ransac.setSimilarityThreshold(0.1f);
    //     // ransac.setMaxCorrespondenceDistance(1.0f);
    //     // ransac.setMaximumIterations(100);


    //     std::cout << "\n";
    //     std::cout << "correspondence size:" <<" " << corrs->size() << std::endl;
    //     std::cout << "\n";

    //     Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();

    //     if(corrs->size() <= corr_num){
    //         Transform(0, 0) = 999999;
    //     }
    //     else{
    //         pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);
    //         // TODO: buzhun
    //         trans->estimateRigidTransformation(*src_cloud_, *tgt_cloud_, *corrs_inliers, Transform);

    //         pcl::PointCloud<pcl::PointXYZ>::Ptr src_final_(new pcl::PointCloud<pcl::PointXYZ>());
    //         pcl::transformPointCloud(*src_cloud_, *src_final_, Transform);
            
    //         if(! checkByICP(src_final_, tgt_cloud_)){
    //             Transform(0, 0) = 999999;
    //         }
    //     }
    //     return Transform;
    // }

    // // this function needs to be reconstructed!!!!!
    // Eigen::Matrix4f getChange2Color(){
    //     // // unitcloud_vec inside
    //     // std::vector<std::vector<int>> curGroup_vec(unitcloud_vec_trans.size());
    //     // for(int i = 0; i < unitcloud_vec_trans.size(); i++){
    //     //     std:: cout << params[i].first << " group inside search:" <<  std::endl;
    //     //         for(int j = 0; j < unitcloud_vec_trans.size(); j++){
    //     //             if(i = j) {continue;}
    //     //             Eigen::Matrix4f tmp_trans = getRegisterMatrix(unitcloud_vec_trans[i], unitcloud_vec_trans[j]);
    //     //             if(tmp_trans(0, 0) == 999999) {continue;}
    //     //             else{curGroup_vec[i].push_back(j);}
    //     //         }   
    //     // }

    //     std::vector<std::pair<std::pair<int, int>, Eigen::Matrix4f>> matches;
    //     for(int i = 0; i < unitcloud_vec_trans.size(); i++){
    //         // search by center
    //         std:: cout << params[i].first << " coarse match by center map search:" <<  std::endl;
    //         std::vector<std::pair<int, float>> tmp_nearUnit_vec = searchUnitByCenter(unitcloud_vec_trans[i]);    

    //         // get matrix
    //         for(int j = 0; j < tmp_nearUnit_vec.size(); j++){
    //             Eigen::Matrix4f tmp_matrix = getRegisterMatrix(unitcloud_vec_trans[i], unitcloud_vec[tmp_nearUnit_vec[j].first]);
               
    //             std::cout << tmp_matrix << std::endl;
                
    //             if(tmp_matrix(0, 0) == 999999) {continue;}
    //             std::pair<int, int> match = std::make_pair(i, tmp_nearUnit_vec[j].first);
    //             matches.push_back(std::make_pair(match, tmp_matrix));    
    //         }
    //     }

    //     // score rank
    //     std::sort(matches.begin(), matches.end(), order);
        
    //     std::cout << 2 << std::endl;
       
    //     static int first_flag = 0;
    //     int match_succeed = 0;
    //     std::vector<std::pair<std::pair<int, int>, Eigen::Matrix4f>> matches_final;
    //     for(int i = 0; i < matches.size(); i++){
    //         if(first_flag == 0){
    //             matches_final.push_back(matches[0]);
    //             first_flag = 1;
    //             match_succeed += 1;
    //         }
    //         else{
    //             if(check(matches[i], matches_final)){

    //                // TODO:  std::cout << 3 <<std::endl;

    //                 matches_final.push_back(matches[i]);
    //                 match_succeed += 1;
    //             }
    //         }

    //         if(match_succeed != unitcloud_vec_trans.size()){
    //             std::cerr << "there is unit having no matched " << std::endl;
    //         }
    //     }

    //     // color red
    //     for(int i = 0; i <  matches_final.size(); i++){
    //         if(((matches_final[i].second(0, 0) * matches_final[i].second(1, 1) * matches_final[i].second(2, 2)) != 1) ||  
    //                 (unitcloud_vec_trans_color[matches_final[i].first.first]->points.size() != unitcloud_vec[matches_final[i].first.second]->points.size())){
    //             for(int j = 0; i < unitcloud_vec_trans_color[matches_final[i].first.first]->points.size(); j++){
    //                 unitcloud_vec_trans_color[matches_final[i].first.first]->points[j].r = 255;
    //                 unitcloud_vec_trans_color[matches_final[i].first.first]->points[j].g = 0;
    //                 unitcloud_vec_trans_color[matches_final[i].first.first]->points[j].b = 0;
    //             }
    //         }
    //     }
    // }
    /*************************************************************************************************************************************************/
    
    std::vector<int> insideMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        tmp_cloud = getModel2World(tgt_cloud_).first;
        pcl::PointXYZ tmp_center = getModel2World(tgt_cloud_).second;

        std::vector<int> inside_vec;
        for(int i = 0; i < unitcloud_vec_trans.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            in_cloud = getModel2World(unitcloud_vec_trans[i]).first;
            pcl::PointXYZ in_center = getModel2World(unitcloud_vec_trans[i]).second;
            if(checkByICP(tmp_cloud, in_cloud)){
                inside_vec.push_back(i);
            }
        }
        return inside_vec;
    }
    
    std::vector<int> outsideMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        tmp_cloud = getModel2World(tgt_cloud_).first;
        pcl::PointXYZ tmp_center = getModel2World(tgt_cloud_).second;

        std::vector<int> outside_vec;  // return outside match idx

        std::vector<std::pair<int, float>> search_vec;
        search_vec = searchUnitByCenter(tgt_cloud_);  // coarse match

        for(int i = 0; i < search_vec.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            out_cloud = getModel2World(unitcloud_vec[search_vec[i].first]).first;
            pcl::PointXYZ out_center = getModel2World(unitcloud_vec[search_vec[i].first]).second;
            if(checkByICP(tmp_cloud, out_cloud)){
                outside_vec.push_back(search_vec[i].first);
            }
        }
        return outside_vec;
    }

    std::pair<Eigen::Matrix4f, float> getMatrixByICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        // kd-icp
        std::pair<Eigen::Matrix4f, float> trans_info;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
        tree1->setInputCloud(src_cloud_);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	    tree2->setInputCloud(tgt_cloud_);
	    icp.setSearchMethodSource(tree1);
	    icp.setSearchMethodTarget(tree2);

        icp.setInputSource(src_cloud_);            
	    icp.setInputTarget(tgt_cloud_);            
	    icp.setTransformationEpsilon(1e-10);   
	    icp.setMaxCorrespondenceDistance(1);  
	    icp.setEuclideanFitnessEpsilon(0.05); 
	    icp.setMaximumIterations(35);          

        trans_info = std::make_pair(icp.getFinalTransformation(), icp.getFitnessScore());
        return trans_info;
    }

    // TODO: dynamic detect
    std::vector<int> getDynamicUnit(){
        std::vector<int> dynamic_vec;
        for(int i = 0; i < unitcloud_vec_trans.size(); i++){   
            std::vector<std::pair<int, std::pair<int, float>>> match_score_vec;
            // inside
            std::vector<int> inside_match_vec;
            std::cout << i + 1 << ".pcd" << " inside match:" << std::endl;
            inside_match_vec = insideMatch(unitcloud_vec_trans[i]);
            for(int p = 0; p < inside_match_vec.size(); p++){
                 std::cout << "icp match: " << inside_match_vec[p] + 1 << ".pcd" << std::endl;
            }
            std::cout << "\n";

            // outsides
            for(int j = 0; j < inside_match_vec.size(); j++){
                std::vector<int> outside_match_vec;
                std::cout << inside_match_vec[j] + 1 << ".pcd" << " outside match" << std::endl;
                outside_match_vec = outsideMatch(unitcloud_vec_trans[inside_match_vec[j]]);

                for(int k = 0; k < outside_match_vec.size(); k++){
                    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    tmp_cloud = getModel2World(unitcloud_vec_trans[inside_match_vec[j]]).first;
                    pcl::PointXYZ tmp_center = getModel2World(unitcloud_vec_trans[inside_match_vec[j]]).second;

                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    out_cloud = getModel2World(unitcloud_vec[outside_match_vec[k]]).first;
                    pcl::PointXYZ out_center = getModel2World(unitcloud_vec[outside_match_vec[k]]).second;

                    std::pair<Eigen::Matrix4f, float> tmp_info;
                    tmp_info = getMatrixByICP(tmp_cloud, out_cloud);

                    // get score
                    // float x, y, z, r, p, y;
                    // Eigen::Affine3f tmp(tmp_info.first);
                    // pcl::getTranslationAndEulerAngles(tmp, x, y, z, r, p, y);

                    float score = std::sqrt(pow((tmp_info.first(0, 3) + tmp_center.x - out_center.x), 2) + pow((tmp_info.first(1, 3) + tmp_center.y - out_center.y), 2) + pow((tmp_info.first(2, 3) + tmp_center.z - out_center.z), 2));
                    tmp_info.first(0, 3) = tmp_info.first(0, 3) + tmp_center.x - out_center.x;
                    tmp_info.first(1, 3) = tmp_info.first(1, 3) + tmp_center.y - out_center.y;
                    tmp_info.first(2, 3) = tmp_info.first(2, 3) + tmp_center.z - out_center.z;

                    std::pair<int, float> tmp_pair = std::make_pair(outside_match_vec[k], score);
                    std::pair<int, std::pair<int, float>> tmp_pairh = std::make_pair(inside_match_vec[j], tmp_pair);

                    match_score_vec.push_back(tmp_pairh);  
                }
            }

            std::sort(match_score_vec.begin(), match_score_vec.end(), order1);
            if(match_score_vec.size() == 0){
                std::cerr << "unit" << " " << i << " " <<  "match error" << std::endl;
            }
            else{
                while(match_score_vec.size() != 0){
                    if(match_score_vec.back().first == i){
                        dynamic_vec.push_back(i);
                        break;
                    }
                    else{
                        match_score_vec.pop_back();
                        continue;
                    }
                }
            }   
        }
        return dynamic_vec;
    }

    void getChange2Color(){
        std::vector<int> color_vec;
        color_vec = getDynamicUnit();
        for(int i = 0; i < color_vec.size(); i++){
            std::cout << "dynamic unit " << color_vec[i] + 1 << ".pcd" << std::endl;
            for(int j = 0; j < unitcloud_vec_trans_color[color_vec[i]]->points.size(); j++){
                unitcloud_vec_trans_color[color_vec[i]]->points[j].r = 255;
                unitcloud_vec_trans_color[color_vec[i]]->points[j].g = 0;
                unitcloud_vec_trans_color[color_vec[i]]->points[j].b = 0;
            }
        }
    }

    bool checkByICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud_){
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	    icp.setInputSource(src_cloud_);         
	    icp.setInputTarget(tgt_cloud_);       
	    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());
        icp.setTransformationEpsilon(1e-5);
        icp.setMaxCorrespondenceDistance(0.1);
        icp.setEuclideanFitnessEpsilon(0.01);
        icp.setMaximumIterations(10);
	    icp.align(*final); 

        if(icp.hasConverged() && (icp.getFitnessScore() < icp_fitness)) {return true;}
        else {return false;}                       
    }

    void getMap(){
        for(int i = 0; i < unitcloud_vec.size(); i++){
            *map_cloud += *unitcloud_vec[i];
        }
        
        for(int i = 0; i < unitcloud_vec_trans.size(); i++){
            *map_cloud_trans += *unitcloud_vec_trans[i];
        }
    }

    // TODO: map reconstruction and color view
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

        // TODO: color2view
        change2Color();

        getChange2Color();

        for(int i = 0; i < unitcloud_vec_trans_color.size(); i++){
            *map_surround_trans_color += *unitcloud_vec_trans_color[i];
        }
    }

    void change2Color(){
        // unitcloud_vec_trans
        for(int i = 0; i < unitcloud_vec_trans.size(); i++){
            for(int j =0; j < unitcloud_vec_trans[i]->points.size(); j++){
                pcl::PointXYZRGB tmp_rgb;
                tmp_rgb.x = unitcloud_vec_trans[i]->points[j].x;
                tmp_rgb.y = unitcloud_vec_trans[i]->points[j].y;
                tmp_rgb.z = unitcloud_vec_trans[i]->points[j].z;
                tmp_rgb.r = 0;
                tmp_rgb.g = 0;
                tmp_rgb.b = 0;
                unitcloud_vec_trans_color[i]->points.push_back(tmp_rgb);
            }
        }

        // map_surround_trans without unit
        for(int i = 0; i < map_surround_trans->points.size(); i++){
            pcl::PointXYZRGB tmp_rgb;
            tmp_rgb.x = map_surround_trans->points[i].x;
            tmp_rgb.y = map_surround_trans->points[i].y;
            tmp_rgb.z = map_surround_trans->points[i].z;
            tmp_rgb.r = 0;
            tmp_rgb.g = 0;
            tmp_rgb.b = 0;
            map_surround_trans_color->points.push_back(tmp_rgb);
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

        if(pcl::io::savePCDFile(save_path + "map_surround_trans_color.pcd", *map_surround_trans_color) != -1)
            std::cout << "map surround trans color has been saved" << std::endl;
    }

    void allocateMemory(){
        
        unitCenter_map.reset(new pcl::PointCloud<pcl::PointXYZ>());

        map_surround.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_surround_trans.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_surround_trans_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        unitcloud_vec.resize(iterm);
        for(int i = 0; i < iterm; i++){
            unitcloud_vec[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        }
        map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        

        unitcloud_vec_trans.resize(iterm);
        for(int i = 0; i < iterm; i++){
            unitcloud_vec_trans[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        }
        unitcloud_vec_trans_color.resize(iterm);
        for(int i = 0; i < iterm; i++){
            unitcloud_vec_trans_color[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        }
        map_cloud_trans.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr unitCenter_map;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_surround;  // init with units and no color
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_surround_trans; // init without uints and on color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_surround_trans_color; // trans with units and color  -> view

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> unitcloud_vec; // init uints
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud; // just init unit map

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> unitcloud_vec_trans; // trans units
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> unitcloud_vec_trans_color; // trans uints with color

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_trans; // just trans units map without color
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
    map_generate.getCenterMap();
    
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
