//
//  ofxSACIAhelper.h
//  bifurcation_26
//
//  Created by Stephan Schulz on 2016-03-23.
//
//

//Sample Consensus Initial Alignment (SAC-IA) registration routine


#ifndef bifurcation_26_ofxSACIAhelper_h
#define bifurcation_26_ofxSACIAhelper_h

//http://www.pcl-users.org/Different-results-by-using-icp-align-and-getfinaltransformation-td4021967.html#a4022076
//http://pointclouds.org/documentation/tutorials/template_alignment.php
//http://answers.ros.org/question/40257/alignment-problem-with-template-and-pointcloud/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/filters/passthrough.h> //for removing distant points
#include <pcl/filters/voxel_grid.h> //for downsampling point cloud



class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
    
    FeatureCloud () :
    search_method_xyz_ (new SearchMethod),
    normal_radius_ (0.02f),
    feature_radius_ (0.02f)
    {}
    
    ~FeatureCloud () {}
    
    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
        xyz_ = xyz;
        processInput ();
    }
    
    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
        xyz_ = PointCloud::Ptr (new PointCloud);
        pcl::io::loadPCDFile (pcd_file, *xyz_);
        processInput ();
    }
    
    
    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
        return (xyz_);
    }
    
    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
        return (normals_);
    }
    
    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
        return (features_);
    }
    
protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
        computeSurfaceNormals ();
        computeLocalFeatures ();
    }
    
    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
        normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
        
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud (xyz_);
        norm_est.setSearchMethod (search_method_xyz_);
        norm_est.setRadiusSearch (normal_radius_);
        norm_est.compute (*normals_);
    }
    
    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
        features_ = LocalFeatures::Ptr (new LocalFeatures);
        
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud (xyz_);
        fpfh_est.setInputNormals (normals_);
        fpfh_est.setSearchMethod (search_method_xyz_);
        fpfh_est.setRadiusSearch (feature_radius_);
        fpfh_est.compute (*features_);
    }
    
private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;
    
    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
public:
    
    // A struct for storing alignment results
    struct Result
    {
        float fitness_score;
        Eigen::Matrix4f final_transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
//    TemplateAlignment () :
//    min_sample_distance_ (5.0f), //0.050f),
//    max_correspondence_distance_ (1.0f), //0.01f*0.01f); // = truncate the error with an upper limit of 1 cm
//    nr_iterations_ (500)
    TemplateAlignment (
    float min_sample_distance_  = 5.0f , //0.050f),
    float max_correspondence_distance_  =  1.0f , //0.01f*0.01f); // = truncate the error with an upper limit of 1 cm
    int nr_iterations_  = 500)
    {
        // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
        sac_ia_.setMinSampleDistance (min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
        sac_ia_.setMaximumIterations (nr_iterations_);
    }
    
    ~TemplateAlignment () {}
    
    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
        target_ = target_cloud;
        sac_ia_.setInputTarget (target_cloud.getPointCloud ());
        sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }
    
    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
        templates_.push_back (template_cloud);
    }
    
    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
//        cout<<"template_cloud.getPointCloud () "<<template_cloud.getPointCloud ()<<endl;
        
        sac_ia_.setInputCloud (template_cloud.getPointCloud ());
        sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());
        
        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align (registration_output);
        
        result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
        result.final_transformation = sac_ia_.getFinalTransformation ();
        
        ofLog()<<"result.fitness_score "<<result.fitness_score;
//        cout<<"result.final_transformation "<<result.final_transformation<<endl;
    }
    
    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
        ofLog()<<"templates_.size () "<<templates_.size ();
        results.resize (templates_.size ());
        for (size_t i = 0; i < templates_.size (); ++i)
        {
            align (templates_[i], results[i]);
        }
    }
    
    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
        // Align all of the templates to the target cloud
        std::vector<Result, Eigen::aligned_allocator<Result> > results;
        alignAll (results);
        
        // Find the template with the best (lowest) fitness score
        float lowest_score = std::numeric_limits<float>::infinity ();
        int best_template = 0;
        for (size_t i = 0; i < results.size (); ++i)
        {
            const Result &r = results[i];
            cout<<"r.fitness_score "<<r.fitness_score<<endl;
            
            if (r.fitness_score < lowest_score)
            {
                lowest_score = r.fitness_score;
                best_template = (int) i;
            }
        }
        
        // Output the best alignment
        result = results[best_template];
        return (best_template);
    }
    
private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;
    
    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};


class ofxSACIAhelper{
public:

    ofMesh SACedMesh;
    ofMatrix4x4 sacMatrix;
    float fitness_score;
    
    std::vector<FeatureCloud> object_templates;
    
    void addObjectTemplate(ofMesh _mesh){
        
//        ofLog()<<"addObjectTemplate() _mesh.size "<<_mesh.getNumVertices();
        
        //http://answers.opencv.org/question/90171/surface_matching-how-to-create-custom-point-cloud/?answer=90199#post-id-90199
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        
        cloud_in->is_dense = false;
        cloud_in->points.resize (_mesh.getNumVertices());
        int cnt = 0;
        for (size_t i = 0; i < cloud_in->points.size (); ++i)
        {
            cloud_in->points[i].x = _mesh.getVertex(i).x;
            cloud_in->points[i].y = _mesh.getVertex(i).y;
            cloud_in->points[i].z = _mesh.getVertex(i).z;
            
//            ofLog()<<i<<" x "<<cloud_in->points[i].x<<" : "<<_mesh.getVertex(i).x;
            cnt++;
        }
        
        FeatureCloud template_cloud;
         template_cloud.setInputCloud(cloud_in);
        object_templates.push_back (template_cloud);
        
//        cout<<"cloud_in "<<*cloud_in;
//        ofLog()<<"object_templates.size "<<object_templates.size()<<" "<<cnt;
        
        
    }
    
    // Load the object templates
    void setObjectTemplates(string _fileName){
        FeatureCloud template_cloud;
        //template_cloud.loadInputCloudbyFile(_fileName);
        object_templates.push_back (template_cloud);
        
        // Load the object templates specified in the object_templates.txt file
        
//        for(int i=0; i<_meshs.size(); i++){
//            FeatureCloud template_cloud;
//            //            template_cloud.loadInputCloud (pcd_filename);
////            template_cloud.loadInputCloud (pcd_filename);
//            object_templates.push_back (template_cloud);
//        }

        
    }
    
    
    ofMatrix4x4 getSAC(ofMesh _mesh, float min_sample_distance,float max_correspondence_distance, int nr_iterations ){
        
 
        
        
        // Load the target cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        cloud->is_dense = false;
        cloud->points.resize (_mesh.getNumVertices());
        int cnt = 0;
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = _mesh.getVertex(i).x;
            cloud->points[i].y = _mesh.getVertex(i).y;
            cloud->points[i].z = _mesh.getVertex(i).z;
            cnt++;
        }

//         ofLog()<<"getSAC() _mesh.size "<<_mesh.getNumVertices()<<" "<<cnt;
//        cout<<"cloud "<<*cloud;
        
//        // Preprocess the cloud by...
//        // ...removing distant points
//        const float depth_limit = 1.0;
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud (cloud);
//        pass.setFilterFieldName ("z");
//        pass.setFilterLimits (0, depth_limit);
//        pass.filter (*cloud);
        
//        // ... and downsampling the point cloud
//        const float voxel_grid_size = 0.005f;
//        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//        vox_grid.setInputCloud (cloud);
//        vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//        //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
//        vox_grid.filter (*tempCloud);
//        cloud = tempCloud;
        
        // Assign to the target FeatureCloud
        FeatureCloud target_cloud;
        //target_cloud.loadInputCloud(ofToDataPath("pcl/person.pcd"));
        target_cloud.setInputCloud (cloud);

        // Set the TemplateAlignment inputs
        //  min_sample_distance_ (5.0f), //0.050f),
//        max_correspondence_distance_ (1.0f), //0.01f*0.01f); // = truncate the error with an upper limit of 1 cm
//        nr_iterations_ (500)
        
        TemplateAlignment template_align(min_sample_distance,max_correspondence_distance,nr_iterations);
//      ofLog()<<"object_templates.size () "<<object_templates.size ();
        for (size_t i = 0; i < object_templates.size (); ++i)
        {
            template_align.addTemplateCloud (object_templates[i]);
        }
        template_align.setTargetCloud (target_cloud);
        
        // Find the best template alignment
        TemplateAlignment::Result best_alignment;
        int best_index = template_align.findBestAlignment (best_alignment);
//        ofLog()<<"best_index "<<best_index;
        const FeatureCloud &best_template = object_templates[best_index];
        
        fitness_score = best_alignment.fitness_score;
        // Print the alignment fitness score (values less than 0.00002 are good)
        printf ("Best fitness score: %f\n", fitness_score);
        
        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
        Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
        

//        printf ("\n");
//        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//        printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//        printf ("\n");
        printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
        
        // Save the aligned template for visualization
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
//        cout<<"transformed_cloud is "<<transformed_cloud;
//        pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud); //needs some other dylib included, not sure which
        
       //Eigen::Matrix4f mat =  best_alignment.final_transformation;
        auto mat = best_alignment.final_transformation;
        
        sacMatrix = ofMatrix4x4(
                    mat(0,0), mat(1,0), mat(2,0), mat(3,0),
                    mat(0,1), mat(1,1), mat(2,1), mat(3,1),
                    mat(0,2), mat(1,2), mat(2,2), mat(3,2),
                    mat(0,3), mat(1,3), mat(2,3), mat(3,3));
        
        
        return sacMatrix;
    }
    
    ofMesh getSACedMesh(ofMesh _mesh){
        SACedMesh.clear();
        SACedMesh.setMode(OF_PRIMITIVE_POINTS);
        
        for(auto v: _mesh.getVertices()){
            SACedMesh.addVertex(v * sacMatrix);
        }
        
        return SACedMesh;
    }

    
};

#endif
