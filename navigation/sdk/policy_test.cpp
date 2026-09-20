#include "nav_policy.hpp"
#include "remote_data.hpp"
#include <limits>
#include <stdexcept>
void require(bool v){if(!v)throw std::runtime_error("navigation policy regression");}
int main(){
 std::array<uint8_t,40> raw{};raw[2]=2;auto r=RemoteData::decode(raw);require(r.keys()==2);
 raw[2]=0;uint32_t half=0x3f000000;for(int j=0;j<4;++j)raw[20+j]=(half>>(8*j))&255;
 r=RemoteData::decode(raw);require(r.ly()==.5f&&r.lx()==0);
 for(int j=0;j<4;++j)raw[20+j]=0;raw[6]=0xc0;raw[7]=0x7f;
 require(RemoteData::decode(raw).keys()!=0);
 using namespace elf_nav;
 require(!state_rejection(.1,0,1,0,0,false));require(!state_rejection(.1,0,3,0,0,true));
 require(state_rejection(.1,0,3,0,0,false));require(!state_rejection(.1,2,1,0,0,false));
 require(!state_rejection(.1,2,3,0,0,true));require(state_rejection(.1,3,1,0,0,false));
 require(state_rejection(.6,0,1,0,0,false));require(state_rejection(-1,0,1,0,0,false));
 require(state_rejection(.1,0,1,.4,0,false));require(state_rejection(.1,0,1,0,std::numeric_limits<double>::quiet_NaN(),false));
 require(!remote_rejection(.1,0,{0,0,0,0}));require(remote_rejection(.6,0,{0,0,0,0}));
 require(remote_rejection(.1,1,{0,0,0,0}));require(remote_rejection(.1,0,{0,.2,0,0}));
 require(remote_rejection(.1,0,{0,0,std::numeric_limits<float>::quiet_NaN(),0}));
}
