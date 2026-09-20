#pragma once
#include <cmath>
#include <array>
namespace elf_nav {
inline const char* state_rejection(double age,unsigned error,unsigned mode,double roll,double pitch,bool moving) {
 if(!std::isfinite(age)||age<0||age>.5)return "robot_state_stale";
 // This Go2 firmware reports exactly 2 while accepting SportClient::Move.
 // The same deployment exception is used by control/elf_go2.cpp.
 if(error!=0&&error!=2)return "robot_error_code_nonzero";
 if(mode!=1&&!(moving&&mode==3))return "robot_mode_rejected";
 if(!std::isfinite(roll)||!std::isfinite(pitch)||std::abs(roll)>=.35||std::abs(pitch)>=.35)return "robot_posture_rejected";
 return nullptr;
}
inline const char* remote_rejection(double age,unsigned keys,const std::array<float,4>&axes) {
 if(!std::isfinite(age)||age<0||age>.5)return "remote_stale";
 if(keys!=0)return "remote_override";
 for(float v:axes)if(!std::isfinite(v)||std::abs(v)>.1)return "remote_override";
 return nullptr;
}
}
