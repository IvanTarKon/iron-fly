#include <cstdio>
#include <cmath>
#include "core/robot.hpp"
int main(){
 gait::Generator gait;
 int reachable=0,clamped=0;
 for(int i=0;i<1000;++i){
  auto feet=gait.update(.02f,30,0,0);
  for(int l=0;l<6;++l){
   auto r=kin::inverse(l,feet.pos[l],{0,0,cfg::STANDING_HEIGHT},{});
   if(!r.ok)continue;
   ++reachable;float q[]={r.coxa,r.femur,r.tibia};
   for(int j=0;j<3;++j){
    float raw=math::rad2deg(q[j])*cfg::DIRECTION[3*l+j]+cfg::TRIM_DEG[3*l+j];
    if(std::fabs(raw-robot::map_servo(l,j,q[j]))>1e-4f)++clamped;
   }
  }
 }
 std::printf("Reachable leg samples: %d/6000; independently clamped joint samples: %d\n",reachable,clamped);
 return reachable==6000 && clamped==0?0:1;
}
