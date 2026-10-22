namespace RealityLog.OVR
{
    /// <summary>Native IMU sampling with the velocity derivative fallback required by OpenXR.</summary>
    public sealed class QuestImuSource
    {
        private OVRPlugin.Vector3f prevVelocity;
        private OVRPlugin.Vector3f prevAngularVelocity;
        private double prevTimestamp;
        private double latestTimestamp;

        public void Reset()
        {
            prevVelocity = default;
            prevAngularVelocity = default;
            prevTimestamp = latestTimestamp = 0;
        }

        public bool TryRead(OVRPlugin.Node node, out OVRPlugin.PoseStatef state)
        {
            state = OVRPlugin.GetNodePoseStateImmediate(node);
            var timestamp = state.Time;
            if (!(timestamp > latestTimestamp)) return false;
            latestTimestamp = timestamp;
            var acc = state.Acceleration;
            var gyro = state.AngularVelocity;
            var vel = state.Velocity;
            if (acc.x == 0 && acc.y == 0 && acc.z == 0 && prevTimestamp > 0)
            {
                double dt = timestamp - prevTimestamp;
                if (dt > 0.0001)
                {
                    state.Acceleration = Derivative(vel, prevVelocity, dt);
                    state.AngularAcceleration = Derivative(gyro, prevAngularVelocity, dt);
                }
            }
            prevVelocity = vel;
            prevAngularVelocity = gyro;
            prevTimestamp = timestamp;
            return true;
        }

        private static OVRPlugin.Vector3f Derivative(OVRPlugin.Vector3f current, OVRPlugin.Vector3f prev, double dt)
        {
            float fDt = (float)dt;
            return new OVRPlugin.Vector3f
            {
                x = (current.x - prev.x) / fDt,
                y = (current.y - prev.y) / fDt,
                z = (current.z - prev.z) / fDt
            };
        }
    }
}
