# Third-party notices

Provenance and licensing for the description assets under `assets/`. First-party assets are
Apache-2.0 like the rest of the repository.

| Tree | Origin | License |
|------|--------|---------|
| `assets/das_gripper_with_vr/` | First-party (Sentient-X DAS/UMI handheld capture gripper, V4, with VR tracking anchors). Moved from the data-pipeline repo (`urdf/DAS_Gripper_with_VR/`). | Apache-2.0 |
| `assets/sxd_arm/` | First-party (the reference 6-DOF arm previously seeded by the data-factory backend from `backend/app/assets/sxd_arm.urdf`). | Apache-2.0 |
| `assets/menagerie/agilex_piper/` | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), Agilex Piper model, upstream commit pinned in `assets/menagerie/menagerie.commit`. | MIT (RosenYin, 2024) — `LICENSE` in-dir |
| `assets/menagerie/i2rt_yam/` | MuJoCo Menagerie, I2RT YAM model, same upstream pin. | MIT (i2rt robotics, 2025) — `LICENSE` in-dir |
| `assets/so101/` | SO-101 arm description (URDF/SRDF + meshes), copied — not moved — from the frozen train recipe baseline `recipes/2026-07-baselines/b6-squint-so101/checkout-pinned/envs/robot/` at train commit `d8ca2fbfb4cef6b6097c71f9ec172c76125a572f` (upstream: the Squint deploy utils, MIT, Abdulaziz Almuzairee 2026; hardware: TheRobotStudio/Seeed SO-ARM101). | MIT — `LICENSE` in-dir |

The `assets/` tree ships in neither wheels nor sdists (see `pyproject.toml` hatch excludes);
consumers resolve it via `sx_embodiments.assets.asset_root()`.
