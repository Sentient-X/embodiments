# Third-party notices

Provenance and licensing for the description assets under `assets/`. First-party assets are
Apache-2.0 like the rest of the repository.

| Tree | Origin | License |
|------|--------|---------|
| `assets/das_gripper_with_vr/` | First-party (Sentient-X DAS/UMI handheld capture gripper, V4, with VR tracking anchors). Moved from the data-pipeline repo (`urdf/DAS_Gripper_with_VR/`). | Apache-2.0 |
| `assets/sxd_arm/` | First-party (the reference 6-DOF arm previously seeded by the data-factory backend from `backend/app/assets/sxd_arm.urdf`). | Apache-2.0 |
| `assets/menagerie/agilex_piper/` | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), Agilex Piper model, upstream commit pinned in `assets/menagerie/menagerie.commit`. | MIT (RosenYin, 2024) — `LICENSE` in-dir |
| `assets/menagerie/aloha/` | MuJoCo Menagerie, ALOHA 2 model, same upstream pin. | BSD-3-Clause — `LICENSE` in-dir |
| `assets/menagerie/franka_emika_panda/` | MuJoCo Menagerie, Franka Emika Panda model, same upstream pin. | Apache-2.0 — `LICENSE` in-dir |
| `assets/menagerie/i2rt_yam/` | MuJoCo Menagerie, I2RT YAM model, same upstream pin. | MIT (i2rt robotics, 2025) — `LICENSE` in-dir |
| `assets/menagerie/rainbow_robotics_rby1/` | MuJoCo Menagerie, Rainbow Robotics RBY1 model family, same upstream pin. | Apache-2.0 — `LICENSE` in-dir |
| `assets/menagerie/unitree_g1/` | MuJoCo Menagerie, Unitree G1 model family, same upstream pin. | BSD-3-Clause — `LICENSE` in-dir |
| `assets/menagerie/universal_robots_ur10e/` | MuJoCo Menagerie, Universal Robots UR10e model, same upstream pin. | BSD-3-Clause — `LICENSE` in-dir |
| `assets/menagerie/universal_robots_ur5e/` | MuJoCo Menagerie, Universal Robots UR5e model, same upstream pin. | BSD-3-Clause — `LICENSE` in-dir |
| `assets/so101/` | SO-101 arm description (URDF/SRDF + meshes), copied — not moved — from the frozen train recipe baseline `recipes/2026-07-baselines/b6-squint-so101/checkout-pinned/envs/robot/` at train commit `d8ca2fbfb4cef6b6097c71f9ec172c76125a572f` (upstream: the Squint deploy utils, MIT, Abdulaziz Almuzairee 2026; hardware: TheRobotStudio/Seeed SO-ARM101). | MIT — `LICENSE` in-dir |
| `assets/yor/` | [YOR](https://github.com/YOR-robot/YOR) robot description, upstream commit pinned in `assets/yor/yor.commit`. | MIT — `LICENSE` in-dir |
| `assets/humanoid_pkg/` | [Sentient-X humanoid_pkg](https://github.com/Sentient-X/humanoid_pkg), retained hardware URDF plus the 28-joint MJCF and their description meshes; upstream commit pinned in `assets/humanoid_pkg/humanoid_pkg.commit`. | BSD-3-Clause — declared by the copied `package.xml` |
| `assets/official/agilex_nero/` | [`agilexrobotics/agx_arm_urdf`](https://github.com/agilexrobotics/agx_arm_urdf), `f6642ce0d7872c686f29c99e9e10cd23d1d49313`, `nero/urdf/nero_description.urdf`. | MIT |
| `assets/official/agilex_piper/` | [`agilexrobotics/piper_ros`](https://github.com/agilexrobotics/piper_ros), `ac41fcbc02295bebd3dd5ac9bc4a9d96f658eb93`, `src/piper_description/urdf/piper_description.urdf`. | MIT |
| `assets/official/aloha/` | Deterministic dual-arm composition of [`Interbotix/interbotix_ros_manipulators`](https://github.com/Interbotix/interbotix_ros_manipulators), `0bb2b0e3846bc85d3577f4f7ce640b48c113e36e`, VX300s xacro. | BSD-3-Clause |
| `assets/official/franka_panda/` | Deterministic xacro expansion of [`frankarobotics/franka_ros`](https://github.com/frankarobotics/franka_ros), `ddd2fffd4106d5ec6a0271cd69342f5a4b013da1`, Panda with hand. | Apache-2.0 |
| `assets/official/rby1/` | [`RainbowRobotics/rby1-sdk`](https://github.com/RainbowRobotics/rby1-sdk), `38df3264c0703ca562408c798209304e90956946`, `models/rby1m_v1.3.urdf`. | Apache-2.0 |
| `assets/official/unitree_g1/` | [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros), `d96d8f63d4e2581c04ba460aba3d08e3d93f6c90`, G1 29-DOF URDF. | BSD-3-Clause |
| `assets/official/universal_robots/` | Deterministic xacro expansions of [`UniversalRobots/Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), `8c2adebd48d8722ec83dd2a06f49049a16b3c9f4`, for UR5e and UR10e. | BSD-3-Clause |
| `assets/official/yor/` | Deterministic URDF projection of the pinned YOR MJCF using `tools/mjcf_to_urdf.py`; the converter preserves the official link/joint tree and controlled joint names. | MIT |

The `assets/` tree ships in wheels as `sx_embodiments/_assets/` and is retained in sdists.
Consumers resolve the installed or editable tree via `sx_embodiments.assets.asset_root()`.
