#pragma once

#include "ofMain.h"

#include "ofxGui.h"


#include "ofxPCL.h"


class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
     void ICP(ofMesh _modelMesh);
    
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	
    void camSavePos();
    void camLoadPos();

    
	// used for viewing the point cloud
	ofEasyCam easyCam;
 
    ofxPCL PCL;
    
    ofMesh templateMesh, blobMesh, ICPedMesh;
    
    ofMatrix4x4 icpTransformation;
   ofMatrix4x4 tempMatrix;
    
    bool bGotNewTemplate;
    
    ofxPanel gui_main;
    ofParameter<float>  min_sample_distance, max_correspondence_distance;
    ofParameter<int> nr_iterations;
};

