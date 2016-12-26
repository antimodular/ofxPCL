#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

    
    blobMesh.load("blobMesh_0.ply");
    blobMesh.setMode(OF_PRIMITIVE_POINTS);
    blobMesh.disableColors();
    
    
    camLoadPos();

    
}

//--------------------------------------------------------------
void ofApp::update() {

    PCL.PCA.getPCA(blobMesh);
//    PCAedMesh = PCL.PCA.getPCAedMesh(blobMesh);

        tempMatrix.makeRotationMatrix(ofRandom(0.1,1), 0, 0, 1);
        for(int i=0; i<blobMesh.getNumVertices(); i++){
            blobMesh.setVertex(i,blobMesh.getVertex(i)*tempMatrix);
        }


}

//--------------------------------------------------------------
void ofApp::draw() {
    

    ofSetColor(255, 255, 255);
    
    easyCam.begin();
    drawPointCloud();

    easyCam.end();
    
    ofDrawBitmapStringHighlight("blue = blobMesh", 400,40);
    ofDrawBitmapStringHighlight("red = orientation and bouding box found by PCA", 400,60);
    
    
    

}

void ofApp::drawPointCloud() {
    glPointSize(3);
    
    ofNoFill();
    ofSetColor(0);
    ofDrawBox(0,0,0,20,20,20);

    
    ofSetColor(0, 0, 255);
    blobMesh.draw();
    
    
    ofSetColor(255, 0, 0);
    PCL.PCA.draw();
    
}

//--------------------------------------------------------------
void ofApp::exit() {

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
