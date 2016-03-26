#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

    
    gui_main.setup("mesh");
    gui_main.setPosition(20,20);
    gui_main.setDefaultHeaderBackgroundColor(ofColor(255,0,0));
    
    gui_main.add(min_sample_distance.set("min_dist",0.1,0.0001,100));
    gui_main.add(max_correspondence_distance.set("max_corres_dist",0.1,0.0001,1000));
    gui_main.add(nr_iterations.set("fps",50,1,1000));
    gui_main.loadFromFile("gui_main.xml");
    
    templateMesh.load("templateMesh.ply");
    templateMesh.setMode(OF_PRIMITIVE_POINTS);
    templateMesh.disableColors();
    
    blobMesh.load("blobMesh_0.ply");
    blobMesh.setMode(OF_PRIMITIVE_POINTS);
    blobMesh.disableColors();
    
    camLoadPos();
    
    bTry = false;
}

//--------------------------------------------------------------
void ofApp::update() {

    if(bTry){
        PCL.SAC.object_templates.clear();
        PCL.SAC.addObjectTemplate(templateMesh);
        // float min_sample_distance,float max_correspondence_distance, int nr_iterations

        icpTransformation = PCL.SAC.getSAC(blobMesh, min_sample_distance, max_correspondence_distance, nr_iterations );
        ICPedMesh = PCL.SAC.getSACedMesh(templateMesh);
        
        ICPedMesh.setMode(OF_PRIMITIVE_POINTS);
        ICPedMesh.disableColors();

    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    

    ofSetColor(255, 255, 255);
    
    easyCam.begin();
    drawPointCloud();

    easyCam.end();
    
    ofDrawBitmapStringHighlight("PCL.SAC.fitness_score "+ofToString(PCL.SAC.fitness_score), 10,430);

    ofDrawBitmapStringHighlight("red = template ", 400,20);
    ofDrawBitmapStringHighlight("blue = blobMesh", 400,40);
    ofDrawBitmapStringHighlight("green = transformedMesh", 400,60);
    
    ofDrawBitmapStringHighlight("press n to start and stop process", 400,100);
    
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
        
        bTry =! bTry;
        
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
