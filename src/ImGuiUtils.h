#pragma once
#include "ofxImGui.h"

//
//  ImGuiUtils.h
//  realsense-osc-tracker
//
//  Created by ole on 11/12/2018.
//

#define IM_ARRAYSIZE(_ARR)  ((int)(sizeof(_ARR)/sizeof(*_ARR)))

ImVec4 from_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a, bool consistent_color)
{
    auto res = ImVec4(r / (float)255, g / (float)255, b / (float)255, a / (float)255);
#ifdef FLIP_COLOR_SCHEME
    if (!consistent_color) return flip(res);
#endif
    return res;
}

ImVec4 operator+(const ImVec4& c, float v)
{
    return ImVec4(
                  std::max(0.f, std::min(1.f, c.x + v)),
                  std::max(0.f, std::min(1.f, c.y + v)),
                  std::max(0.f, std::min(1.f, c.z + v)),
                  std::max(0.f, std::min(1.f, c.w))
                  );
}

namespace ImGui {
    
    bool InputTextFromString(const char* label, string & str, ImGuiInputTextFlags flags = 0, ImGuiTextEditCallback callback = NULL, void* user_data = NULL){
        static char buf[255] = "";
        strcpy(buf, str.c_str());
        if(ImGui::InputText(label, &buf[0], IM_ARRAYSIZE(buf), flags, callback, user_data)){
            str = string(buf);
            return true;
        }
        return false;
    }
    
}
