// Diagnostic of unmodified upstream core; not a hardware controller.
#include <cassert>
#include <cmath>
#include <cstdio>
#include "core/robot.hpp"
int main() {
 hw::Proxy hw; robot::Robot r(hw); r.enable();
 for(int i=0;i<1500 && r.state()!=proto::State::IDLE;++i){r.heartbeat();r.update(.02f);}
 assert(r.state()==proto::State::IDLE);
 std::puts("stand-up with heartbeat: PASS");
 const float commands[][3]={{30,0,0},{-30,0,0},{0,30,0},{0,-30,0},{0,0,10},{20,10,5}};
 for(auto &v:commands){
  for(int i=0;i<150;++i){r.set_velocity(v[0],v[1],v[2]);r.update(.02f);
   for(float q:r.joints().deg)assert(std::isfinite(q));}
  assert(r.state()==proto::State::WALK);
  for(int i=0;i<500 && r.state()!=proto::State::IDLE;++i){r.stop();r.update(.02f);}
  assert(r.state()==proto::State::IDLE);
 }
 std::puts("six movement scenarios and settling: PASS");
 // Explicit public mapping: LF LM LR RF RM RR; output kinematic radians.
 const int map[]={5,4,3,0,1,2};
 std::puts("neutral joint targets (LF LM LR RF RM RR), rad:");
 for(int l:map)for(int j=0;j<3;++j)std::printf("%.6f%c",robot::unmap_servo(l,j,r.joints().deg[3*l+j]),j==2?'\n':' ');
 for(int i=0;i<1000;++i)r.update(.02f);
 assert(r.state()==proto::State::OFF && !hw.powered);
 std::puts("upstream timeout -> sit down -> OFF: PASS (differs from planned STOP)");
}
