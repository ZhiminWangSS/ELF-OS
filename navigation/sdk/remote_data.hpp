#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
// Official SDK example/state_machine/gamepad.hpp: LE keys at 2,
// lx/rx/ry/L2/ly at 4/8/12/16/20. Status reception is not RF-link proof.
struct RemoteData {
 unsigned key=0;std::array<float,5> axis{{0,0,0,0,0}};
 unsigned keys() const{return key;}
 float lx()const{return axis[0];} float rx()const{return axis[1];}
 float ry()const{return axis[2];} float ly()const{return axis[4];}
 static RemoteData decode(const std::array<uint8_t,40>& b){
   RemoteData r;r.key=unsigned(b[2])|(unsigned(b[3])<<8);
   for(int i=0;i<5;++i){uint32_t u=0;for(int j=0;j<4;++j)u|=uint32_t(b[4+i*4+j])<<(8*j);std::memcpy(&r.axis[i],&u,4);
     if(!std::isfinite(r.axis[i])||std::abs(r.axis[i])>1.1f)r.key|=0x10000;
   }
   if(std::abs(r.axis[3])>.1f)r.key|=0x10000;
   return r;
 }
};
