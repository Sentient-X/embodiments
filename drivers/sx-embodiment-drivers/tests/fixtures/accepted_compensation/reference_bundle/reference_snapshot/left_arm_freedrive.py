# AST-only data excerpt from the accepted handoff. Not executable hardware code.
TORQUE_SLEW_RATE_NM_S = 20.0
FRICTION_TORQUE = {'left_shoulder_pitch_joint': 0.0, 'left_shoulder_roll_joint': 0.0, 'left_upper_arm_yaw_joint': 0.2, 'left_elbow_pitch_joint': 0.22, 'left_forearm_yaw_joint': 0.16, 'left_wrist_pitch_joint': 0.0, 'left_wrist_roll_joint': 0.1}

def urdf_to_motor_torque(self, tau_urdf: Sequence[float], dt: float=0.02, dq_motors: Sequence[float] | None=None, states: Sequence[str] | None=None) -> list[float]:
    t_motor = []
    max_delta = TORQUE_SLEW_RATE_NM_S * dt
    for i, m in enumerate(LEFT_ARM_MOTORS):
        sign = self.signs[m.rx_id]
        target_t = sign * tau_urdf[i] * self.gravity_ratio
        if m.joint != J6_NAME and dq_motors is not None and (self.friction_comp > 0):
            is_guiding = states is None or states[i] in ('FREE', 'BRAKE') or self.trackers[i].pure_freedrive
            if is_guiding:
                dq_m = dq_motors[i]
                fric_c = FRICTION_TORQUE.get(m.joint, 0.0) * self.friction_comp
                if m.joint == 'left_elbow_pitch_joint':
                    speed = max(0.0, abs(dq_m) - 0.02)
                    target_t += math.copysign(fric_c * math.tanh(speed / 0.03), dq_m)
                else:
                    target_t += fric_c * math.tanh(dq_m / 0.01)
        if i in (4, 6):
            target_t = min(0.4, max(-0.4, target_t))
        elif i == 5:
            target_t = min(0.75, max(-0.75, target_t))
        effort_clamp = self.max_torque_fraction * min(m.t_max, m.effort_limit)
        target_t = min(effort_clamp, max(-effort_clamp, target_t))
        prev = self.prev_t_ff[i]
        slew_t = min(prev + max_delta, max(prev - max_delta, target_t))
        self.prev_t_ff[i] = slew_t
        t_motor.append(slew_t)
    return t_motor
