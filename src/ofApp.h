#pragma once

#include "ofMain.h"
#include <librealsense2/rs.hpp>
#include "ofxImGui.h"
#include "ofxCv.h"
#include "MeshTracker.hpp"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void keycodePressed(ofKeyEventArgs& e);
    
    
    // TRACKING
    
    rs2::pipeline pipe;
    rs2::device device;
    rs2::pipeline_profile selection;
    rs2::colorizer color_map;
    rs2::frame colored_depth;
    rs2::frame colored_filtered;
    rs2_intrinsics intrinsics;
    
    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    
    rs2::points points;
    rs2::pointcloud pc;
    
    ofNode origin;
    
    ofMesh trackingMesh;
    
    ofCamera trackingCamera;
    
    MeshTracker tracker;
    
    //setup of the virtual room
    
    ofPlanePrimitive floorPlane;
    ofPlanePrimitive wallNegPlane;
    ofPlanePrimitive wallPosPlane;
    ofPlanePrimitive backWallPlane;
    // ofCamera trackingCamera;
    
    // ofxCv::KalmanPosition trackingKalman;
    // MeshTracker tracker;
    
    ofEasyCam cam; // add mouse controls for camera movement
    
    // GUI
    
    bool imGui();
    ofxImGui::Gui gui;
    string title = "Realsense Osc Tracker";
    bool mouseOverGui;
    int guiColumnWidth = 250;
    ImFont* gui_font_header;
    ImFont* gui_font_text;
    ofImage logo;
    GLuint logoID;
    
    void save(string name);
    void load(string name);
    
    // PARAMETER
    
//    ofParameter<bool> pTrackingEnabled{ "Enabled", false};
//    ofParameter<bool> pTrackingVisible{ "Visible", false};
    ofParameter<float> pTrackingTimeout{ "Timeout", 30.0, 0.0, 5*60.0};
    
    ofParameter<glm::vec3> pTrackingStartPosition{ "Start Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    
    ofParameter<glm::vec3> pTrackingCameraPosition{ "Tracking Camera Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    ofParameter<glm::vec3> pTrackingCameraRotation{ "Tracking Camera Rotation", glm::vec3(0.,0.,0.), glm::vec3(-180.,-180.,-180.), glm::vec3(180.,180.,180.)};
    
    ofParameter<glm::vec3> pTrackingBoxPosition{ "Tracking Box Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    ofParameter<glm::vec3> pTrackingBoxRotation{ "Tracking Box Rotation", glm::vec3(0.,0.,0.), glm::vec3(-180.,-180.,-180.), glm::vec3(180.,180.,180.)};
    ofParameter<glm::vec3> pTrackingBoxSize{ "Tracking Box Size", glm::vec3(1.,1.,1.), glm::vec3(0.,0.,0.), glm::vec3(10.,10.,10.)};
    
    ofParameter<glm::vec3> pFloorPlanePosition{ "Floor Plane Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
   
    ofParameter<glm::vec3> pWallNegXPlanePosition{ "Wall -X Plane Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    
    ofParameter<glm::vec3> pWallPosXPlanePosition{ "Wall +X Plane Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    
        ofParameter<glm::vec3> pBackWallPlane{ "Back Wall Plane Position", glm::vec3(0.,0.,0.), glm::vec3(-10.,-10.,-10.), glm::vec3(10.,10.,10.)};
    
    ofParameterGroup pgRoot{"Tracking", pTrackingTimeout, pTrackingCameraPosition, pTrackingCameraRotation, pTrackingBoxPosition, pTrackingBoxRotation, pTrackingBoxSize, pTrackingStartPosition, pFloorPlanePosition, pWallNegXPlanePosition, pWallPosXPlanePosition, pBackWallPlane};
    
};
