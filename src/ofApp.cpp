#include "ofApp.h"
#include "ImGuiUtils.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    // WINDOW
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofSetWindowTitle(title);
    
    trackingMesh.setMode(OF_PRIMITIVE_POINTS);
    
    cropVerticesQueue = dispatch_queue_create("Crop Vertices", DISPATCH_QUEUE_CONCURRENT);
        
    //REALSENSE
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
        
        // FILTERS
        
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.95f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.1f);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 65.0f);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, 7);
        
    } catch (exception e){
        ofLogError("SETUP", "No realsense camera found");
    }
    
    ofAddListener(ofGetWindowPtr()->events().keyPressed, this,
                  &ofApp::keycodePressed);
    
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    gui_font_text = io.Fonts->AddFontFromFileTTF(ofToDataPath("fonts/OpenSans-Regular.ttf").c_str(), 16.0f);
    gui_font_header = io.Fonts->AddFontFromFileTTF(ofToDataPath("fonts/OpenSans-Regular.ttf").c_str(), 22.0f);
    
    gui.setup();
    
    logo.load("images/logo.png");
    logoID = gui.loadImage(logo);
    
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(14.,14.);
    style.WindowRounding = 4.0f;
    style.ChildWindowRounding = 4.;
    style.FramePadding = ImVec2(8, 4);
    style.FrameRounding = 4;
    style.ItemSpacing = ImVec2(6, 5);
    style.ItemInnerSpacing = ImVec2(7, 4);
    style.IndentSpacing = 18;
    style.ScrollbarRounding = 2;
    
    style.Colors[ImGuiCol_Text]                  = ImVec4(0.90f, 0.90f, 0.90f, 1.00f);
    style.Colors[ImGuiCol_TextDisabled]          = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    style.Colors[ImGuiCol_WindowBg]              = ImVec4(0.025f, 0.025f, 0.025f, 0.85f);
    style.Colors[ImGuiCol_ChildWindowBg]         = ImVec4(0.00f, 0.00f, 0.00f, 0.22f);
    style.Colors[ImGuiCol_PopupBg]               = ImVec4(0.05f, 0.05f, 0.05f, 0.90f);
    style.Colors[ImGuiCol_Border]                = ImVec4(0.70f, 0.70f, 0.70f, 0.29f);
    style.Colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    style.Colors[ImGuiCol_FrameBg]               = ImVec4(0.07f, 0.09f, 0.10f, 0.40f);
    style.Colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.00f, 0.45f, 0.78f, 0.39f);
    style.Colors[ImGuiCol_FrameBgActive]         = ImVec4(0.65f, 0.74f, 0.81f, 0.33f);
    style.Colors[ImGuiCol_TitleBg]               = ImVec4(0.11f, 0.13f, 0.15f, 1.00f);
    style.Colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(0.11f, 0.13f, 0.15f, 1.00f);
    style.Colors[ImGuiCol_TitleBgActive]         = ImVec4(0.24f, 0.29f, 0.32f, 1.00f);
    style.Colors[ImGuiCol_MenuBarBg]             = ImVec4(0.11f, 0.13f, 0.15f, 0.80f);
    style.Colors[ImGuiCol_ScrollbarBg]           = ImVec4(0.05f, 0.07f, 0.08f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.21f, 0.24f, 0.26f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.31f, 0.34f, 0.36f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabActive]   = ImVec4(0.11f, 0.13f, 0.15f, 1.00f);
    style.Colors[ImGuiCol_ComboBg]               = ImVec4(0.04f, 0.04f, 0.05f, 1.00f);
    style.Colors[ImGuiCol_CheckMark]             = ImVec4(0.90f, 0.90f, 0.90f, 0.57f);
    style.Colors[ImGuiCol_SliderGrab]            = ImVec4(1.00f, 1.00f, 1.00f, 0.13f);
    style.Colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.00f, 0.45f, 0.78f, 1.00f);
    style.Colors[ImGuiCol_Button]                = ImVec4(0.24f, 0.30f, 0.35f, 0.18f);
    style.Colors[ImGuiCol_ButtonHovered]         = ImVec4(0.00f, 0.45f, 0.78f, 0.39f);
    style.Colors[ImGuiCol_ButtonActive]          = ImVec4(0.00f, 0.45f, 0.78f, 1.00f);
    style.Colors[ImGuiCol_Header]                = ImVec4(0.24f, 0.30f, 0.35f, 0.18f);
    style.Colors[ImGuiCol_HeaderHovered]         = ImVec4(0.00f, 0.45f, 0.78f, 0.39f);
    style.Colors[ImGuiCol_HeaderActive]          = ImVec4(0.00f, 0.45f, 0.78f, 1.00f);
    style.Colors[ImGuiCol_Column]                = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    style.Colors[ImGuiCol_ColumnHovered]         = ImVec4(0.70f, 0.60f, 0.60f, 1.00f);
    style.Colors[ImGuiCol_ColumnActive]          = ImVec4(0.90f, 0.70f, 0.70f, 1.00f);
    style.Colors[ImGuiCol_ResizeGrip]            = ImVec4(1.00f, 1.00f, 1.00f, 0.30f);
    style.Colors[ImGuiCol_ResizeGripHovered]     = ImVec4(1.00f, 1.00f, 1.00f, 0.60f);
    style.Colors[ImGuiCol_ResizeGripActive]      = ImVec4(1.00f, 1.00f, 1.00f, 0.90f);
    style.Colors[ImGuiCol_CloseButton]           = ImVec4(0.00f, 0.00f, 0.00f, 0.50f);
    style.Colors[ImGuiCol_CloseButtonHovered]    = ImVec4(0.00f, 0.45f, 0.78f, 0.39f);
    style.Colors[ImGuiCol_CloseButtonActive]     = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
    style.Colors[ImGuiCol_PlotLines]             = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    style.Colors[ImGuiCol_PlotLinesHovered]      = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_PlotHistogram]         = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_PlotHistogramHovered]  = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_TextSelectedBg]        = ImVec4(0.00f, 0.56f, 1.00f, 0.35f);
    style.Colors[ImGuiCol_ModalWindowDarkening]  = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);
    
    // PARAMETERS
    
    load("default");
    
    // Visualisation planes
    
    floorPlane.setParent(origin);
    wallPosPlane.setParent(origin);
    wallNegPlane.setParent(origin);
    
    // MESH TRACKER
    
    trackingCamera.setParent(origin);
    trackingCamera.setupPerspective();
    // realsense camera frustrum
    trackingCamera.setAspectRatio(848.0/480.0);
    trackingCamera.setFov(86.0);
    trackingCamera.setNearClip(0.1);
    trackingCamera.setFarClip(50.0);
    tracker.setup(3, pTrackingStartPosition, trackingCamera, origin );
    
    //GUI
    
    cam.setupPerspective();
    cam.setParent(origin);
    cam.setFarClip(50000.0);
    cam.setNearClip(0.01);

    
}

//--------------------------------------------------------------
void ofApp::update(){

    cam.setPosition(pTrackingBoxPosition.get().x+(pTrackingBoxSize.get().x/1.75),
                    pTrackingBoxPosition.get().y+pTrackingBoxSize.get().y,
                    (pTrackingBoxPosition.get().z+pTrackingBoxSize.get().z)*2.0);
    cam.lookAt(tracker, glm::vec3(0.0,-1.0,0.0));

    //FIXME: Glitches in update/draw using instruments?
    
    //TRACKER
    trackingCamera.setPosition(pTrackingCameraPosition);
    trackingCamera.setOrientation(pTrackingCameraRotation);
    tracker.setPosition(pTrackingBoxPosition);
    tracker.setOrientation(pTrackingBoxRotation);
    tracker.set(pTrackingBoxSize.get().x, pTrackingBoxSize.get().y, pTrackingBoxSize.get().z);
    tracker.startingPoint.setGlobalPosition(pTrackingStartPosition);
    tracker.camera.setGlobalPosition(trackingCamera.getGlobalPosition());
    tracker.camera.setGlobalOrientation(trackingCamera.getGlobalOrientation());
    tracker.camera.setScale(trackingCamera.getScale());
    
    const auto cameraGlobalMat = trackingCamera.getGlobalTransformMatrix();
    const auto trackerInverse = glm::inverse(tracker.getGlobalTransformMatrix());
    
    if(selection){
        
        rs2::frameset frames;
        
        if(pipe.poll_for_frames(&frames)){
            
            // Get depth data from camera
            auto depthFrame = frames.get_depth_frame();
            
            rs2::frame filteredFrame = depthFrame; // make a copy
            // Note the concatenation of output/input frame to build up a chain
            filteredFrame = dec_filter.process(filteredFrame);
            filteredFrame = spat_filter.process(filteredFrame);
            filteredFrame = temp_filter.process(filteredFrame);
            
            points = pc.calculate(filteredFrame);
            
            // Create oF mesh
            trackingMesh.clear();
            int n = points.size();
            if(n>0){
                const rs2::vertex * vs = points.get_vertices();
                for(int i=0; i<n; i++){
                    if(vs[i].z>0.5){ // save time on skipping the closest ones
                        const rs2::vertex v = vs[i];
                        glm::vec3 v3(v.x,-v.y,-v.z);
                        glm::vec4 cameraVec(v3, 1.0);
                        glm::vec4 globalVec = cameraGlobalMat * cameraVec;
                        
                        auto inversedVec = trackerInverse * globalVec;
                        glm::vec3 trackerVec = glm::vec3(inversedVec) / inversedVec.w;
                        
                        if(fabs(trackerVec.x) < tracker.getWidth()/2.0 &&
                           fabs(trackerVec.y) < tracker.getHeight()/2.0 &&
                           fabs(trackerVec.z) < tracker.getDepth()/2.0){
                            
                            int wasAdded = tracker.addVertex(v3);
                            
                            trackingMesh.addVertex(v3);
                            
                            ofFloatColor c;
                            if(wasAdded == 0){
                                c = ofFloatColor::lightGray;
                            } else if (wasAdded == 1){
                                c = ofFloatColor::cyan;
                            } else if (wasAdded == 2){
                                c= ofFloatColor::green;
                            } else if (wasAdded == 3){
                                c = ofFloatColor::blueSteel;
                            }
                            
                            trackingMesh.addColor(c);
                            
                        }
                    }
                }
                tracker.update();
            }
        }
    }
    
    float roomWidth = fmax(fabs(pWallNegXPlanePosition.get().x), fabs(pWallPosXPlanePosition.get().x)) * 2.0;
    float roomDepth = pFloorPlanePosition.get().z * 2.0;
    float roomHeight = pBackWallPlane.get().y * 2.0;
    
    //setting the parameters for the plane
    floorPlane.set(roomWidth, roomDepth);   ///dimensions for width and height in pixels
    floorPlane.setOrientation(glm::vec3(90.,0.,0.));
    floorPlane.setGlobalPosition(pFloorPlanePosition); /// position in x y z
    //floorPlane.setResolution(2, 2);
    
    wallNegPlane.set(roomDepth,roomHeight);
    wallNegPlane.setGlobalPosition(pWallNegXPlanePosition);
    wallNegPlane.setOrientation(glm::vec3(0.,90.,0.));
    //wallNegPlane.setResolution(2, 2);
    
    wallPosPlane.set(roomDepth,roomHeight);
    wallPosPlane.setGlobalPosition(pWallPosXPlanePosition);
    wallPosPlane.setOrientation(glm::vec3(0.,90.,0.));
    //wallPosPlane.setResolution(2, 2);
    
    backWallPlane.set(roomWidth,roomHeight);
    backWallPlane.setGlobalPosition(pBackWallPlane);
    backWallPlane.setOrientation(glm::vec3(0.,0.,0.));
    //wallNegPlane.setResolution(2, 2);
    
    
    // Create oF mesh
    trackingMesh.clear();
    int n = points.size();
    if(n>0){
        
        const rs2::vertex * vs = points.get_vertices();
        
        bool vertsActive[ n ];
        bool *vertsActivePointer = vertsActive;
        
        dispatch_apply(n, cropVerticesQueue, ^(size_t i) {
            
            const rs2::vertex & v = vs[i];
            
            vertsActivePointer[i] = false;
            
            if(v.z>0.5){ // save time on skipping the closest ones
                
                glm::vec3 v3(v.x,-v.y,-v.z);
                glm::vec4 cameraVec(v3, 1.0);
                glm::vec4 globalVec = cameraGlobalMat * cameraVec;
                
                auto inversedVec = trackerInverse * globalVec;
                glm::vec3 trackerVec = glm::vec3(inversedVec) / inversedVec.w;
                
                if(fabs(trackerVec.x) < tracker.getWidth()/2.0 &&
                   fabs(trackerVec.y) < tracker.getHeight()/2.0 &&
                   fabs(trackerVec.z) < tracker.getDepth()/2.0){
                    vertsActivePointer[i] = true;
                }
            }
        });
                
        for(int i=0; i<n; i++){
            
            const rs2::vertex & v = vs[i];
            
            glm::vec3 v3(v.x,-v.y,-v.z);
            
            ofFloatColor c(0.0,64.0);
            
            if(vertsActive[i]){
                
                int wasAdded = tracker.addVertex(v3);
                
                if(wasAdded == 0){
                    c = ofFloatColor::lightGray;
                } else if (wasAdded == 1){
                    c = ofFloatColor::cyan;
                } else if (wasAdded == 2){
                    c= ofFloatColor::green;
                } else if (wasAdded == 3){
                    c = ofFloatColor::blueSteel;
                }
                
            }
            
            trackingMesh.addVertex(v3);
            trackingMesh.addColor(c);
            
        }
        
        
        
        tracker.update();
        // OSC
        if(oscTrackingSender.getHost() != pOscTrackingRemoteHost.get() ||
           oscTrackingSender.getPort() != pOscTrackingRemotePort.get()
           ){
            //cout << "setting oscTrackingSender ";
            oscTrackingSender.clear();
            oscTrackingSender.setup(pOscTrackingRemoteHost.get(), pOscTrackingRemotePort.get());
            //cout << oscTrackingSender.getHost() << ":";
            //cout << oscTrackingSender.getPort() << endl;
        }
        
        //osc
        
        for (auto & head : tracker.heads) {
            if(head.isTrackingOrLost()){
                
                //OSC sending head position
                auto headPosCoord = head.getGlobalPosition();

                ofxOscMessage oscMessage;
                
                //int idAddress = head.id;
                string idAddress = ofToString(head.id);
                oscMessage.setAddress("/tracker/"+idAddress+"/head/position");
                oscMessage.addFloatArg(headPosCoord.x);
                oscMessage.addFloatArg(headPosCoord.y);
                oscMessage.addFloatArg(headPosCoord.z);
                oscTrackingSender.sendMessage(oscMessage);

                oscMessage.setAddress("/tracker/"+idAddress+"/floor/position");
                oscMessage.addFloatArg(headPosCoord.x);
                oscMessage.addFloatArg(0.0);
                oscMessage.addFloatArg(headPosCoord.z);
                oscTrackingSender.sendMessage(oscMessage);

            }
            
        }
    }
}


//--------------------------------------------------------------
void ofApp::draw(){
    
    // TODO: Fix gui to the left
    // TODO: Viewport for camera in right side
    // TODO: Senisble camera position at startup
    
    ofBackground(33);
    
    cam.begin(); {
        
        //ofScale(ofGetWidth());  //1024 pixels
        
        ofDrawAxis(1.0); //Global axis
        
        ofSetColor(255, 64);
        
        ofEnableDepthTest();
        
        ofSetColor(90, 255);
        
        floorPlane.drawWireframe();
        backWallPlane.drawWireframe();
        wallNegPlane.drawWireframe();
        wallPosPlane.drawWireframe();
        
        ofSetColor(255, 64);
        
        floorPlane.draw();
        backWallPlane.draw();
        wallNegPlane.draw();
        wallPosPlane.draw();
        
        if(pTrackingVisible){
            ofDisableDepthTest();
            trackingCamera.transformGL();
            trackingMesh.draw();
            trackingCamera.restoreTransformGL();
            ofEnableDepthTest();
            trackingCamera.drawFrustum();
        }
        tracker.draw();
        
        
    } cam.end();
    
    
    
    // GUI
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofFill();
    ofSetColor(255,255);
    this->mouseOverGui = this->imGui();
    if (this->mouseOverGui) {
        cam.disableMouseInput();
    } else {
        cam.enableMouseInput();
    }
    
    
}



//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

void ofApp::keycodePressed(ofKeyEventArgs& e){
    if(e.hasModifier(OF_KEY_CONTROL)){
        if(e.keycode == 'F'){
            ofToggleFullscreen();
        }
    }
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

void ofApp::save(string name){
    ofJson j;
    ofSerialize(j, pgRoot);
    ofSaveJson("settings/" + name + ".json", j);
}

void ofApp::load(string name){
    ofJson j = ofLoadJson("settings/" + name + ".json");
    ofDeserialize(j, pgRoot);
}

bool ofApp::imGui()
{
    //TODO: Merge GUI code from Ole
    
    auto mainSettings = ofxImGui::Settings();
    
    ofDisableDepthTest();
    
    this->gui.begin();
    {
        ImGui::PushFont(gui_font_text);
        
        ImGuiWindowFlags window_flags = 0;
        window_flags |= ImGuiWindowFlags_NoCollapse;
        
        if (ofxImGui::BeginWindow(title.c_str(), mainSettings, window_flags)){
            
            // DFV Header
            
            ImGui::Columns(2, "TitleColumns", false);
            ImGui::PushFont(gui_font_header);
            ImGui::TextUnformatted(title.c_str());
            ImGui::PopFont();
            ImGui::TextUnformatted("(c) den frie vilje 2018");
            ImGui::Text("FPS %.3f", ofGetFrameRate());
            int logoSize = 60;
            ImGui::SetColumnOffset(1, ImGui::GetWindowContentRegionMax().x - (logoSize + 7));
            ImGui::NextColumn();
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.8);
            ImGui::Image((ImTextureID)(uintptr_t)logoID, {static_cast<float>(logoSize),static_cast<float>(logoSize)});
            ImGui::PopStyleVar();
            ImGui::Columns(1);
            
            
            ImGui::Separator();
            
            if(ImGui::Button("Load")){
                load("default");
            } ImGui::SameLine();
            if(ImGui::Button("Save")){
                save("default");
            }
            
            static bool guiShowTest;
            ImGui::Checkbox("Show Test Window", &guiShowTest);
            if(guiShowTest)
                ImGui::ShowTestWindow();
            
            
            ImGui::Separator();
            
            int columnOffset = 200;
            
            if(ofxImGui::BeginTree("Head Tracker OSC", mainSettings)){
                
                ImGui::Columns(2, "HeadTrackerOSCColumns", false);
                
                string strHost = pOscTrackingRemoteHost.get();
                if(ImGui::InputTextFromString("Remote Host", strHost, ImGuiInputTextFlags_CharsNoBlank)){
                    pOscTrackingRemoteHost.set(strHost);
                }
                
                ImGui::SetColumnOffset(1, ImGui::GetWindowContentRegionMax().x - columnOffset);
                
                ImGui::NextColumn();
                
                string strPort = ofToString(pOscTrackingRemotePort.get());
                if(ImGui::InputTextFromString("Remote Port", strPort, ImGuiInputTextFlags_CharsDecimal)){
                    pOscTrackingRemotePort.set(ofToInt(string(strPort)));
                }
                
                ImGui::Columns(1);
                
                ofxImGui::EndTree(mainSettings);
            }
            
            if(ofxImGui::BeginTree("qLab OSC", mainSettings)){
                
                ImGui::Columns(2, "qLabOscColumns", false);
                
                string strHost = pOscQlabRemoteHost.get();
                if(ImGui::InputTextFromString("Remote Host", strHost, ImGuiInputTextFlags_CharsNoBlank)){
                    pOscQlabRemoteHost.set(strHost);
                }
                
                ImGui::SetColumnOffset(1, ImGui::GetWindowContentRegionMax().x - columnOffset);
                
                ImGui::NextColumn();
                
                string strPort = ofToString(pOscQlabRemotePort.get());
                if(ImGui::InputTextFromString("Remote Port", strPort, ImGuiInputTextFlags_CharsDecimal)){
                    pOscQlabRemotePort.set(ofToInt(string(strPort)));
                }
                
                ImGui::Columns(1);
                
                ImGui::Columns(2, "qLabOscRemote", false);
                
                ImGui::SetColumnOffset(1, ImGui::GetWindowContentRegionMax().x - columnOffset);
                
                ImGui::NextColumn();
                
                string strPortReply = ofToString(pOscQlabReplyPort.get());
                if(ImGui::InputTextFromString("Reply Port", strPortReply, ImGuiInputTextFlags_CharsDecimal)){
                    pOscQlabReplyPort.set(ofToInt(string(strPortReply)));
                }
                
                ImGui::Columns(1);
                
                ofxImGui::EndTree(mainSettings);
            }
            
            
            /*
             for (auto pg : pgRoot){
             ofxImGui::AddGroup(pg->castGroup(), mainSettings);
             }
             */
            
            ofxImGui::AddGroup(pgTracking, mainSettings);
            
            ofxImGui::EndWindow(mainSettings);
        }
        
    }
    
    ImGui::PopFont();
    
    this->gui.end();
    
    return mainSettings.mouseOverGui;
}
