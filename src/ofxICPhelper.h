//
//  ofxICPhelper.h
//  bifurcation_26
//
//  Created by Stephan Schulz on 2016-03-18.
//
//

#ifndef bifurcation_26_ofxICPhelper_h
#define bifurcation_26_ofxICPhelper_h

////----taken from ofxPCL.h file
//// file io
//#include <pcl/io/pcd_io.h>
//
//// transform
//#include <pcl/common/transforms.h>
//
//// thresold
//#include <pcl/filters/passthrough.h>
//
//// outlier removal
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
//
//// segmentation
//#include <pcl/sample_consensus/model_types.h>
//
//// downsample
//#include <pcl/filters/voxel_grid.h>
//
//// segmentation
//#include <pcl/ModelCoefficients.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/filters/extract_indices.h>
//
//// cluster extraction
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/extract_clusters.h>
//
//// triangulate
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/grid_projection.h>
//#include <pcl/Vertices.h>
//
//// mls
//#include <pcl/surface/mls.h>
//#include <pcl/io/pcd_io.h>
//
//#include <pcl/surface/organized_fast_mesh.h>
//#include <pcl/features/integral_image_normal.h>
//
////------taken from ofxPCL utility.h file
//#include <pcl/common/io.h>
//
////taken from types.h file
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//
////-----taken from tree.h
//// octree
//#include <pcl/octree/octree.h>
//
//// kdtree
//#include <pcl/search/pcl_search.h>

//-------taken from http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
#include <pcl/registration/icp.h>




class ofxICPhelper {
    
public:
    

    float convergeScore;
    bool hasConverged;
    
    ofMatrix4x4 icpMatrix;
    GLfloat glMat[16];
    
    
    float roll, pitch, yaw;
    
    ofMesh ICPedMesh;
    
    
    ofMatrix4x4 getICP(ofMesh _liveMesh, ofMesh _lastMesh){
        
        //http://pointclouds.org/documentation/tutorials/interactive_icp.php
        //http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
        
        hasConverged = false;
        convergeScore = 0;
        
//        ofLog()<<"_lastMesh.getNumVertices() "<<_lastMesh.getNumVertices()<<" : "<<_liveMesh.getNumVertices();
        
        if(_lastMesh.getNumVertices() > 0 && _liveMesh.getNumVertices() > 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
            
            // Fill in the CloudIn data
            //    cloud_in->width    = _modelMesh.getNumVertices();
            //    cloud_in->height   = 1;
            cloud_in->is_dense = false;
            cloud_in->points.resize (_lastMesh.getNumVertices());
            for (size_t i = 0; i < cloud_in->points.size (); ++i)
            {
                cloud_in->points[i].x = _lastMesh.getVertex(i).x;
                cloud_in->points[i].y = _lastMesh.getVertex(i).y;
                cloud_in->points[i].z = _lastMesh.getVertex(i).z;
            }
            
            
            //        std::cout << "Saved " << cloud_in->points.size () << " data points to input:"<< std::endl;
            
            //        for (size_t i = 0; i < cloud_in->points.size (); ++i){
            //            std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;
            //        }
            //    *cloud_out = *cloud_in;
            
            cloud_out->is_dense = false;
            cloud_out->points.resize (_liveMesh.getNumVertices());
            for (size_t i = 0; i < cloud_out->points.size (); ++i)
            {
                cloud_out->points[i].x = _liveMesh.getVertex(i).x;
                cloud_out->points[i].y = _liveMesh.getVertex(i).y;
                cloud_out->points[i].z = _liveMesh.getVertex(i).z;
            }
            
//            cout<<"cloud_in "<<*cloud_in<<endl;
//            cout<<"cloud_out "<<*cloud_out<<endl;
    
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputCloud(cloud_in);
            icp.setInputTarget(cloud_out);
            

            //        icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
            
            //        icp.setMaxCorrespondenceDistance(0.01);
            //        icp.setMaximumIterations(10);
            //        icp.setTransformationEpsilon(1e-2);
            //        icp.setEuclideanFitnessEpsilon(1e-5);
            
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
//            cout<<"Final "<<Final;
            
            
            hasConverged = icp.hasConverged();
            convergeScore = icp.getFitnessScore();
//            std::cout << "\n\nhas converged:" <<  hasConverged << " score: " << convergeScore << std::endl;
           
                    std::cout << icp.getFinalTransformation() << std::endl;
            
            //        Eigen::Matrix4f mat = icp.getFinalTransformation();
            auto mat = icp.getFinalTransformation();
            
            
            //        finalMatrix = ofMatrix4x4(
            //                                  mat(0,0), mat(1,0), mat(2,0), mat(3,0),
            //                                  mat(0,1), mat(1,1), mat(2,1), mat(3,1),
            //                                  mat(0,2), mat(1,2), mat(2,2), mat(3,2),
            //                                  0, 0, 0, 1);
            
            icpMatrix = ofMatrix4x4(
                                      mat(0,0), mat(1,0), mat(2,0), mat(3,0),
                                      mat(0,1), mat(1,1), mat(2,1), mat(3,1),
                                      mat(0,2), mat(1,2), mat(2,2), mat(3,2),
                                      mat(0,3), mat(1,3), mat(2,3), mat(3,3));
   
            //        ofLog()<<"finalMatrix \n"<<finalMatrix;
            
            // Executing the transformation
            //        pcl::transformPointCloud (*cloud_in, *cloud_out, mat);
            
 
            Eigen::Transform<float, 3, Eigen::Affine> tROTA(mat);
            float x,y,z;
            pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
            
            // ofMatrix4x4				openGL
            //							 X	 Y   Z    T
            // [0]  [1]  [2]  [3]		[0] [4] [8]  [12]
            // [4]  [5]  [6]  [7]		[1] [5] [9]  [13]
            // [8]  [9]  [10] [11]		[2] [6] [10] [14]
            // [12] [13] [14] [15]		[3] [7] [11] [15]
//            glMat[0] = mat(0,0); glMat[4] = mat(0,1); glMat[8] = mat(0,2); glMat[12] = mat(0,3);
//            glMat[1] = mat(1,0); glMat[5] = mat(1,1); glMat[9] = mat(1,2); glMat[13] = mat(1,3);
//            glMat[2] = mat(2,0); glMat[6] = mat(2,1); glMat[10] = mat(2,2); glMat[14] = mat(2,3);
//            glMat[3] = mat(3,0); glMat[7] = mat(3,1); glMat[11] = mat(3,2); glMat[15] = mat(3,3);
            
            if(hasConverged == true && convergeScore < 200){
                ICPedMesh.clear();
                ICPedMesh.setMode(OF_PRIMITIVE_POINTS);
                for(auto v: _lastMesh.getVertices()){
                    ICPedMesh.addVertex(v * icpMatrix);
                }
                
                //            ICPedMesh_red.clear();
                //            ICPedMesh_red.setMode(OF_PRIMITIVE_POINTS);
                //            for (size_t i = 0; i < Final.points.size (); ++i)
                //            {
                //                ICPedMesh_red.addVertex(ofVec3f(Final.points[i].x,Final.points[i].y,Final.points[i].z));
                //            }
                
            }else{
                ofLogNotice("ofxICPhelper")<<"hasConverged == false. convergeScore = "<<convergeScore;
            }
            
        }
        
//        return ICPedMesh;
        //http://stackoverflow.com/questions/18956151/icp-transformation-matrix-interpretation
        //pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
       
        return icpMatrix;
    }

    
};


#endif
