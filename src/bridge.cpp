// Reuse upstream bridge without duplicating the transport or firmware lifecycle.
#include "../vendor/Hexapod-Simulation/bridge/sim_bridge.cpp"
extern "C" {
int fw_pose_rejected(Firmware *f) { return f->robot.pose_rejected() ? 1 : 0; }
unsigned fw_rejected_poses(Firmware *f) { return f->robot.rejected_poses(); }
}
