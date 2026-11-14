# AST-only data excerpt from the accepted handoff. Not executable hardware code.
LEFT_ARM_JOINTS = (JointKinematics(name='left_shoulder_pitch_joint', parent_link='chest_link', child_link='left_shoulder_pitch_link', xyz=np.array([-0.059, 0.23097, 0.013], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, 0.965926, 0.258815], dtype=np.float64), lower_limit=-2.64, upper_limit=2.89), JointKinematics(name='left_shoulder_roll_joint', parent_link='left_shoulder_pitch_link', child_link='left_shoulder_roll_link', xyz=np.array([0.03085, 0.05825, 0.0156], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([1.0, 0.0, 0.0], dtype=np.float64), lower_limit=-0.387, upper_limit=1.202), JointKinematics(name='left_upper_arm_yaw_joint', parent_link='left_shoulder_roll_link', child_link='left_upper_arm_yaw_link', xyz=np.array([-0.03085, 0.01268, -0.1423], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, 0.0, -1.0], dtype=np.float64), lower_limit=-5.12, upper_limit=2.356), JointKinematics(name='left_elbow_pitch_joint', parent_link='left_upper_arm_yaw_link', child_link='left_elbow_pitch_link', xyz=np.array([0.0, -0.03825, -0.0665], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, -1.0, 0.0], dtype=np.float64), lower_limit=-2.975, upper_limit=0.087), JointKinematics(name='left_forearm_yaw_joint', parent_link='left_elbow_pitch_link', child_link='left_forearm_yaw_link', xyz=np.array([0.0, 0.03825, -0.067], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, 0.0, -1.0], dtype=np.float64), lower_limit=-6.83, upper_limit=1.88), JointKinematics(name='left_wrist_pitch_joint', parent_link='left_forearm_yaw_link', child_link='left_wrist_pitch_link', xyz=np.array([0.0, 0.0, -0.123], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, -1.0, 0.0], dtype=np.float64), lower_limit=-2.13, upper_limit=2.44), JointKinematics(name='left_wrist_roll_joint', parent_link='left_wrist_pitch_link', child_link='left_wrist_roll_link', xyz=np.array([0.0, 0.0, -0.0955], dtype=np.float64), rpy=np.array([0.0, 0.0, 0.0], dtype=np.float64), axis=np.array([0.0, 0.0, -1.0], dtype=np.float64), lower_limit=-2.1, upper_limit=2.1))

def forward_kinematics(self, q: Sequence[float]) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    p_joints: list[np.ndarray] = []
    z_axes: list[np.ndarray] = []
    p_coms: list[np.ndarray] = []
    q_geom = list(q)
    q_geom[3] = float(q[3]) + math.pi / 2.0
    T = np.eye(4, dtype=np.float64)
    for i in range(7):
        joint = self.joints[i]
        link = self.links[i]
        r_mat = rpy_to_rotation_matrix(joint.rpy[0], joint.rpy[1], joint.rpy[2])
        t_fixed = np.eye(4, dtype=np.float64)
        t_fixed[:3, :3] = r_mat
        t_fixed[:3, 3] = joint.xyz
        T = T @ t_fixed
        axis_world = T[:3, :3] @ joint.axis
        p_joint = T[:3, 3].copy()
        r_joint = rodrigues_rotation(joint.axis, float(q_geom[i]))
        t_joint = np.eye(4, dtype=np.float64)
        t_joint[:3, :3] = r_joint
        T = T @ t_joint
        p_com = (T @ np.append(link.com, 1.0))[:3]
        p_joints.append(p_joint)
        z_axes.append(axis_world)
        p_coms.append(p_com)
    return (p_joints, z_axes, p_coms)
