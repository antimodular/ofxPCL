//
//  ofxPCAhelper.h
//  bifurcation_26
//
//  Created by Stephan Schulz on 2016-03-22.
//
//

//http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
//https://github.com/otherlab/pcl/blob/master/test/common/test_pca.cpp

#ifndef bifurcation_26_ofxPCAhelper_h
#define bifurcation_26_ofxPCAhelper_h

//#include "ofMain.h"

#include <pcl/common/pca.h>

#include <pcl/common/common.h>
//#include <pcl/common/eigen.h>
//#include <pcl/common/transforms.h>

class ofxPCAhelper{
public:
    
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::PCA<pcl::PointXYZ> pca;
    
    ofVec3f pca_translation;
    ofMatrix4x4 pcaMatrix;
    ofMatrix4x4 pca_rotMatrix;
    ofMatrix4x4 pca_transMatrix;
    
    ofMesh PCAedMesh;
    
    double width,height,depth;
    
    ofMatrix4x4 toOf(Eigen::Matrix3f mat){
        ofMatrix4x4 result = ofMatrix4x4(
                                              mat(0,0), mat(1,0), mat(2,0), 0,
                                              mat(0,1), mat(1,1), mat(2,1), 0,
                                              mat(0,2), mat(1,2), mat(2,2), 0,
                                              0,0,0,1);

        return result;
    }
    
    ofMatrix4x4 getPCA(ofMesh _liveMesh){
        // Load the target cloud PCD file
  
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        
        cloud_in->is_dense = false;
        cloud_in->points.resize (_liveMesh.getNumVertices());
        for (size_t i = 0; i < cloud_in->points.size (); ++i)
        {
            cloud_in->points[i].x = _liveMesh.getVertex(i).x;
            cloud_in->points[i].y = _liveMesh.getVertex(i).y;
            cloud_in->points[i].z = _liveMesh.getVertex(i).z;
        }
        
        
//        std::cout << "PointCloud before filtering has: " << cloud_in->points.size ()
//        << " data points." << std::endl; //*
        
        pcl::PCA<pcl::PointXYZ> pca;
        pcl::PointCloud<pcl::PointXYZ> proj;
        
        pca.setInputCloud (cloud_in);
        pca.project (*cloud_in, proj);
        
        pcl::PointXYZ proj_min;
        pcl::PointXYZ proj_max;
        pcl::getMinMax3D (proj, proj_min, proj_max);
        
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pca.reconstruct (proj_min, min);
        pca.reconstruct (proj_max, max);
//        std::cout << " min.x= " << min.x << " max.x= " << max.x << " min.y= " <<
//        min.y << " max.y= " << max.y << " min.z= " << min.z << " max.z= " << max.z
//        << std::endl;
        
        //Rotation of PCA
        Eigen::Matrix3f rot_mat = pca.getEigenVectors ();
        pca_rotMatrix = toOf(rot_mat);
        
              //translation of PCA
        Eigen::Vector3f cl_translation = pca.getMean().head(3);
        pca_translation = ofVec3f(cl_translation.x(),cl_translation.y(),cl_translation.z());
        pca_transMatrix.makeTranslationMatrix(pca_translation);
        
        /*
        Eigen::Matrix3f affine_trans;
        std::cout << rot_mat << std::endl;
        
        //Reordering of principal components
        affine_trans.col(0) <<
        (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
        affine_trans.col(1) << rot_mat.col(0);
        affine_trans.col(2) << rot_mat.col(1);
        //affine_trans.col(3) << cl_translation,1;
        
        std::cout << affine_trans << std::endl;
        
        Eigen::Quaternionf rotation = Eigen::Quaternionf (affine_trans);
        Eigen::Vector4f t = pca.getMean();
        
        Eigen::Vector3f translation = Eigen::Vector3f (t.x(), t.y(), t.z());
        
//        pca_translation = ofVec3f(t.x(), t.y(), t.z());
*/

        pcaMatrix = pca_transMatrix.getInverse() * pca_rotMatrix;
         width = fabs(proj_max.x-proj_min.x);
         height = fabs(proj_max.y-proj_min.y);
         depth = fabs(proj_max.z-proj_min.z);
        
        return pcaMatrix;
    }
    
    void draw(){
        ofPushMatrix();
        ofMultMatrix(pca_rotMatrix*pca_transMatrix);
        ofSetColor(255,0,0);
        ofNoFill();
        
        ofDrawLine(ofVec3f(0,0,0),ofVec3f(200,0,0));
        
        ofDrawBox(ofVec3f(0,0,0),width,height,depth);
        ofPopMatrix();
        
    }
    
    ofMesh getPCAedMesh(ofMesh _mesh){
        PCAedMesh.clear();
        PCAedMesh.setMode(OF_PRIMITIVE_POINTS);
        
        for(auto v: _mesh.getVertices()){
            PCAedMesh.addVertex(v * pcaMatrix);
        }

        return PCAedMesh;
    }
    

};
#endif
