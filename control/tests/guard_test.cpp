#include "guard.hpp"
#include <limits>
#include <iostream>
#include <string>
int main() {
  auto require=[](bool v) { if(!v) throw std::runtime_error("guard invariant failed"); };
  elf::validate({.2,0,1.25}); elf::validate({0,-.35,.5});
  for(auto s : {elf::Step{.21,0,1}, {-.1,0,1}, {.1,.1,1}, {0,0,1}, {.1,0,2},
                {.1,0,-1}, {std::numeric_limits<double>::quiet_NaN(),0,1}}) {
    bool rejected=false; try { elf::validate(s); } catch(...) { rejected=true; }
    require(rejected);
  }
  require(elf::healthy(.1,0,1,0,0,true));
  require(elf::healthy(.1,0,3,0,0,false));
  require(!elf::healthy(.1,0,3,0,0,true));
  require(!elf::healthy(.6,0,1,0,0,false));
  require(elf::healthy(.1,2,3,0,0,false));
  require(elf::healthy(.1,2,1,0,0,true));
  require(!elf::healthy(.6,2,3,0,0,false));
  require(!elf::healthy(.1,2,7,0,0,false));
  require(!elf::healthy(.1,2,3,.4,0,false));
  for (unsigned code : {1u,3u,6u,1002u})
    require(!elf::healthy(.1,code,3,0,0,false));
  require(std::string(elf::rejection(.1,3,3,0,0,false)) == "robot_error_code_nonzero");
  require(std::string(elf::rejection(.6,0,1,0,0,false)) == "state_stale");
  require(std::string(elf::rejection(.1,0,3,0,0,true)) == "mode_not_allowed");
  require(std::string(elf::rejection(.1,0,1,.35,0,false)) == "excessive_tilt");
  require(!elf::healthy(.1,0,7,0,0,false));
  require(!elf::healthy(.1,0,1,.5,0,false));
  require(!elf::healthy(.1,0,1,0,std::numeric_limits<double>::quiet_NaN(),false));
  std::cout << "guard checks passed\n";
}
