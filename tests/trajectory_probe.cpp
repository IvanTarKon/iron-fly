// White-box diagnostics only: inspect requested feet BEFORE upstream clamp.
// No firmware implementation changes; all standard headers precede the macro.
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <cstdint>
#include "core/config.hpp"
#include "core/gait.hpp"
#include "core/kinematics.hpp"
#include "communication/hardware.hpp"
#define private public
#include "core/robot.hpp"
#undef private

int main() {
  // Reachable neutral target versus an explicitly unreachable target.
  gait::Generator neutral;
  math::Vec3 feet[cfg::N_LEGS];
  for(int leg=0;leg<6;++leg) feet[leg]=neutral.neutral(leg);
  robot::ServoAngles solved;
  if(!robot::resolve(feet,{0,0,cfg::STANDING_HEIGHT},{},solved)) return 2;
  feet[0].x=10000;
  if(robot::resolve(feet,{0,0,cfg::STANDING_HEIGHT},{},solved)) return 3;
  int unreachable=0, clipped=0, fk_bad=0, samples=0, startup_clipped=0, rejected=0, unsafe=0;
  float max_error=0;
  const float commands[][3]={{30,0,0},{-30,0,0},{0,30,0},{0,-30,0},
                            {0,0,math::rad2deg(.15f)},{0,0,-math::rad2deg(.15f)}};
  for (int g=0;g<3;++g) for (auto &v:commands) {
    hw::Proxy hw; robot::Robot r(hw);
    r.set_gait(static_cast<gait::GaitId>(g)); r.enable();
    for(int tick=0;tick<700;++tick) {
      r.heartbeat();
      r.set_velocity(tick>=300 && tick<600?v[0]:0,
                     tick>=300 && tick<600?v[1]:0,
                     tick>=300 && tick<600?v[2]:0);
      const auto previous=r.joints();
      r.update(.02f);
      if(r.pose_rejected()) {
        ++rejected;
        for(int i=0;i<18;++i) if(previous.deg[i]!=r.joints().deg[i]) ++unsafe;
      }
      if(r.state()==proto::State::SETUP && r.seq_[r.seq_idx_]==robot::Robot::Step::Curl) continue;
      for(int leg=0;leg<6;++leg) {
        auto ik=kin::inverse(leg,r.leg_[leg],r.body_pos_,r.body_rpy_);
        if(!ik.ok) { ++unreachable; continue; }
        float angles[]={ik.coxa,ik.femur,ik.tibia};
        for(int j=0;j<3;++j) {
          float raw=math::rad2deg(angles[j])*cfg::DIRECTION[3*leg+j]+cfg::TRIM_DEG[3*leg+j];
          if(std::fabs(raw-robot::map_servo(leg,j,angles[j]))>1e-3f) {
            ++clipped;
            if(!r.pose_rejected()) ++unsafe;
            if(tick<300) ++startup_clipped;
          }
        }
        auto foot=kin::forward(leg,ik.coxa,ik.femur,ik.tibia,r.body_pos_,r.body_rpy_,true);
        float error=(foot-r.leg_[leg]).norm();
        if(error>max_error) max_error=error;
        if(error>.01f) ++fk_bad;
        ++samples;
      }
    }
  }
  std::printf("samples=%d unreachable=%d clipped=%d (startup=%d) FK_bad=%d max_FK_mm=%g\n",
              samples,unreachable,clipped,startup_clipped,fk_bad,max_error);
  std::printf("rejected whole poses=%d unsafe writes=%d\n",rejected,unsafe);
  return unreachable||unsafe||fk_bad||clipped!=startup_clipped?1:0;
}
