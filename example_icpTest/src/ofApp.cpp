#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

    
//    gui_main.setup("mesh");
//    gui_main.setPosition(20,20);
//    gui_main.setDefaultHeaderBackgroundColor(ofColor(255,0,0));
//    
//    gui_main.add(min_sample_distance.set("min_dist",0.1,0.0001,100));
//    gui_main.add(max_correspondence_distance.set("max_corres_dist",0.1,0.0001,1000));
//    gui_main.add(nr_iterations.set("fps",50,1,1000));
//    gui_main.loadFromFile("gui_main.xml");
    

    
    blobMesh.load("blobMesh_0.ply");
    blobMesh.setMode(OF_PRIMITIVE_POINTS);
    blobMesh.disableColors();
    
    
    //for the purpose of this example
    //not loading a template but rather generate one that is only slightly offset from the original blobMesh
    tempMatrix.makeRotationMatrix(10, 0, 1, 0);
    for(auto v: blobMesh.getVertices()){
        templateMesh.addVertex(v * tempMatrix);
    }
    //templateMesh.load("templateMesh.ply");
    templateMesh.setMode(OF_PRIMITIVE_POINTS);
    templateMesh.disableColors();
    
    camLoadPos();
    
//            ICPedMesh.setMode(OF_PRIMITIVE_POINTS);
//            ICPedMesh.disableColors();
    
    bGotNewTemplate = true;
    
}

//--------------------------------------------------------------
void ofApp::update() {

    if(bGotNewTemplate == true){
        bGotNewTemplate = false;
        icpTransformation = PCL.ICP.getICP(blobMesh, templateMesh);
        ICPedMesh = PCL.ICP.ICPedMesh;
    }else{
    
        icpTransformation = PCL.ICP.getICP(blobMesh, ICPedMesh);
        ICPedMesh = PCL.ICP.ICPedMesh;
        
        //for the purpose of this example
        //making motion of the blobMesh by adding a random rotation to it
        tempMatrix.makeRotationMatrix(ofRandom(10), 0, 0, 1);
        for(int i=0; i<blobMesh.getNumVertices(); i++){
            blobMesh.setVertex(i,blobMesh.getVertex(i)*tempMatrix);
        }

    }

}

//--------------------------------------------------------------
void ofApp::draw() {
    

    ofSetColor(255, 255, 255);
    
    easyCam.begin();
    drawPointCloud();

    easyCam.end();
    
    ofDrawBitmapStringHighlight("bGotNewTemplate "+ofToString(bGotNewTemplate), 10,370);
    ofDrawBitmapStringHighlight("PCL.ICP.convergeScore "+ofToString(PCL.ICP.convergeScore), 10,400);
    
    ofDrawBitmapStringHighlight("red = template ", 400,20);
    ofDrawBitmapStringHighlight("blue = blobMesh", 400,40);
    ofDrawBitmapStringHighlight("green = transformedMesh", 400,60);
    
    gui_main.draw();

}

void ofApp::drawPointCloud() {
    glPointSize(3);
    
    ofNoFill();
    ofSetColor(0);
    ofDrawBox(0,0,0,20,20,20);
    
    ofSetColor(255, 0, 0);
    templateMesh.draw();
    
    ofSetColor(0, 0, 255);
    blobMesh.draw();
    
    
    ofSetColor(0, 255, 0);
    ICPedMesh.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
   gui_main.saveToFile("gui_main.xml");
}

void ofApp::camLoadPos(){
    //grabcamera setup
    //grabcam position
    //read in default 3d view
    ofMatrix4x4 tempPosition;
    
    ofFile inFile;
    inFile.open("savePose.mat", ofFile::ReadOnly, true);
    inFile.read((char*) tempPosition.getPtr(), sizeof(float) * 16);
    inFile.close();
    easyCam.setTransformMatrix(tempPosition);
    
    cout<<"load camera position"<<endl;
    
}

void ofApp::camSavePos(){
    //bCamSavePos = !bCamSavePos;
    ofMatrix4x4 tempPosition = easyCam.getGlobalTransformMatrix();
    
    ofFile outFile;
    outFile.open("savePose.mat", ofFile::WriteOnly, true);
    outFile.write((char*) tempPosition.getPtr(), sizeof(float) * 16);
    outFile.close();
    
    cout<<"save camera position"<<endl;
    
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    if(key == 's'){
        camSavePos();
    }
    
    if(key == 'n'){
    
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    
}
