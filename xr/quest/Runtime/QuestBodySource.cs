using System;

namespace RealityLog.OVR
{
    /// <summary>Owns the Meta full-body tracker and its native output buffer.</summary>
    public sealed class QuestBodySource : IDisposable
    {
        public const int JointCount = 84;
        private OVRPlugin.BodyState state = new OVRPlugin.BodyState
        {
            JointLocations = new OVRPlugin.BodyJointLocation[JointCount]
        };
        public bool Started { get; private set; }

        public bool Start()
        {
            if (Started) return true;
            Started = OVRPlugin.StartBodyTracking2(OVRPlugin.BodyJointSet.FullBody);
            if (Started) OVRPlugin.RequestBodyTrackingFidelity(OVRPlugin.BodyTrackingFidelity2.High);
            return Started;
        }

        public bool TryRead(out OVRPlugin.BodyState sample)
        {
            bool available = OVRPlugin.GetBodyState4(OVRPlugin.Step.Render, OVRPlugin.BodyJointSet.FullBody, ref state);
            sample = state;
            return available;
        }

        public void Dispose()
        {
            if (!Started) return;
            OVRPlugin.StopBodyTracking();
            Started = false;
        }
    }
}
