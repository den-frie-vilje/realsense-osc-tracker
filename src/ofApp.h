#pragma once

#include "ofMain.h"
#include <librealsense2/rs.hpp>


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
    
    ofMesh trackingMesh;
    
    
    // ofCamera trackingCamera;
    
    // ofxCv::KalmanPosition trackingKalman;
    // MeshTracker tracker;
    
    ofEasyCam cam; // add mouse controls for camera movement
    
};
