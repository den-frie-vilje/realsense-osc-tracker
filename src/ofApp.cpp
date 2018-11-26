#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    trackingMesh.setMode(OF_PRIMITIVE_POINTS);
    
    rs2::config cfg;
    
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_ANY, 60);
    
    try {
        
        selection = pipe.start(cfg);
        
        // Find first depth sensor (devices can have zero or more then one)
        auto depth_sensor = selection.get_device().first<rs2::depth_sensor>();
        
        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        }
        if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f); // Enable autoexposure
        }
        
        /* manual exposure options
         
         if (depth_sensor.supports(RS2_OPTION_GAIN))
         {
         depth_sensor.set_option(RS2_OPTION_GAIN, 32.f);
         }
         if (depth_sensor.supports(RS2_OPTION_EXPOSURE))
         {
         depth_sensor.set_option(RS2_OPTION_EXPOSURE, 4000.f);
         }
         */
        
        if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
        {
            // Query min and max values:
            auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        }
        
        auto scale =  depth_sensor.get_depth_scale();
        
        auto stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
        if (auto video_stream = stream.as<rs2::video_stream_profile>())
        {
            try
            {
                //If the stream is indeed a video stream, we can now simply call get_intrinsics()
                intrinsics = video_stream.get_intrinsics();
                
                auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
                auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
                rs2_distortion model = intrinsics.model;
                /*
                 std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
                 std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
                 std::cout << "Distortion Model        : " << model << std::endl;
                 std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
                 intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
                 */
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
            }
        }
        
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.95f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.1f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 65.0f);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, 7);
        
        
        
    } catch (exception e){
        ofLogError("SETUP", "No realsense camera found");
    }
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    // Get depth data from camera
    auto frames = pipe.wait_for_frames();
    auto depthFrame = frames.get_depth_frame();
    
    rs2::frame filteredFrame = depthFrame; // make a copy
    // Note the concatenation of output/input frame to build up a chain
    filteredFrame = dec_filter.process(filteredFrame);
    filteredFrame = spat_filter.process(filteredFrame);
    filteredFrame = temp_filter.process(filteredFrame);
    
    points = pc.calculate(filteredFrame);
    
    // Create oF mesh
    trackingMesh.clear();
    
    int numberPoints = points.size();
    
    if(numberPoints!=0){
        // get a pointer to the vertice array
        const rs2::vertex * vs = points.get_vertices();
        
        for(int i=0; i<numberPoints; i++){
            if(vs[i].z>0.5){ // save time on skipping the closest ones
                const rs2::vertex v = vs[i];
                glm::vec3 v3(v.x,v.y,v.z);
                trackingMesh.addVertex(v3);
                trackingMesh.addColor(ofFloatColor::blueSteel);
            }
        }
        
    }
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    cam.begin();
    trackingMesh.draw();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
