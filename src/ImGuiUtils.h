#pragma once

//
//  ImGuiUtils.h
//  realsense-osc-tracker
//
//  Created by ole on 11/12/2018.
//

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
