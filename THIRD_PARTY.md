# Third-party source inventory

Sources retrieved on 2026-11-06. The pinned firmware now has one recorded local adaptation; see below.

| Component | Commit | Use |
|---|---|---|
| https://github.com/ggldnl/Hexapod-Firmware | a56d397074c383c454e02bee507a7e242474e22a | Comparison checkout, GPL-3.0, see LICENSE.md |
| https://github.com/ggldnl/Hexapod-Simulation | fc03ebf8b222031c40dc003c95f96d7f96302cf9 | Original bridge and physics model integration, see LICENSE.md |
| Firmware pinned by Simulation | 82d94e01df3e977c624bb671db4933b5fa1e572c | Actual diagnostic build, GPL-3.0 |
| https://github.com/ggldnl/Hexapod-Controller | 0f68c0c15e1d194675c294022baa5efbade28f6a | Original Python command client, see repository license |
| https://github.com/ggldnl/Hexapod-Hardware | f8ee8084d48470283d5ca23394d5a4b9590ad9e8 | Original URDF and meshes, see repository license |

core_probe.cpp uses core/robot.hpp (Robot), its gait and kinematics transitively,
and hw::Proxy. sim_probe.py uses the original Firmware/SimTransport,
HexapodClient and BulletInterface. These reuse the upstream implementation; the local whole-pose validation patch is recorded below. Distribution of a derivative must retain upstream
licenses; this repository does not relicense third-party material.

Controller license verified: GPL-3.0. Hardware license verified: CERN-OHL-W-2.0.
Simulation license: GPL-3.0. config/demo.yml is a copy of the Controller YAML with
an added demonstration disclaimer; original numerical values retained.

## Local adaptation, 2026-11-06

`patches/0001-reject-whole-pose.patch` modifies only the pinned Firmware's
`src/core/robot.hpp`: reject out-of-range whole poses and expose rejection diagnostics.
The commit remains 82d94e01df3e977c624bb671db4933b5fa1e572c; the working tree includes
this GPL-3.0 derivative patch. `src/bridge.cpp` includes the upstream simulation
bridge and adds read-only diagnostic C functions; no protocol or gait replacement.
Other vendor sources and upstream tests are unchanged.

Fusion and the additional suggested projects are not installed or incorporated.
Their licensing and suitability have not been audited in this implementation.
