# Go2 + YAM Pro v1

`urdf/combined_urdf.py` combines the bundled Unitree Go2 quadruped and I2RT
YAM Pro v1 arm. `main.py` selects `Go2_YAM(reference_pose="standing", arm_joints=6)`.
The resulting floating-base model has 25 configuration coordinates, 24 velocity
coordinates, 12 leg joints, and 6 arm joints. The two gripper fingers are locked.

## Source models

The files under `models/` are unmodified upstream assets, with their licenses:

| Local directory | Upstream source | Revision |
| --- | --- | --- |
| `models/go2_description` | [Unitree Go2 description](https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/go2_description) (`urdf/go2_description.urdf` and the referenced `dae/` meshes) | `5994d4faef0a9cadd3287f8de0199a67eeb2a259` |
| `models/yam_pro/v1` | [I2RT YAM Pro v1](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058/i2rt/robot_models/arm/yam_pro/v1) (`yam_pro.urdf`, `yam_pro.xml`, and the referenced `assets/` meshes) | `120c3c81400171174604e503943f8d1ebc891058` |

The builder resolves meshes to absolute paths, renames Go2's root from `base`
to `base_link` for the MPC, and retains the arm's separate `base` link. It writes
a combined URDF with a filename derived from its contents so changes to either
source, the mount, or the repository location cannot reuse an older model.

## Model assumptions

- The fixed arm mount retains the existing translation `(0.190, 0, 0.1027)` m
  and rotation `(0, 0, 0)` rad relative to the Go2 base. This is a project
  assumption, not a mounting calibration supplied by either manufacturer.
- The standing pose puts Go2's foot frames at ground height with thigh/calf
  angles `0.7` / `-1.5` rad. The front/rear force target is estimated from the
  combined model's nominal center of mass, replacing the B2 load split.
- I2RT's URDF specifies `1 Nm` effort for every arm joint. The generated model
  instead uses the `10 Nm` bounds in its companion `yam_pro.xml` simulation
  model. The URDF's `1 rad/s` arm velocity limits are retained. These are
  simulation settings, not validated continuous hardware limits.
- The end-effector frame is the upstream `gripper` link. No payload or additional
  mounting-bracket inertia is included.
- `arm_joints` accepts integers from 0 through 6. Unused arm joints and both
  gripper joints are locked at their neutral positions.
- The default Fatrop iteration cap is 100. The previous cap of 10 stopped
  before this model's initial tracking problems converged; this change trades
  solver time for convergence and does not establish real-time performance.

## Validation

In the `wb-mpc` environment, run:

```bash
python -m unittest discover -s tests -v
```

The integration checks load the actual meshes and verify the model topology,
mass, joint ordering, reference pose, reduction, and operation outside the
repository's working directory.
