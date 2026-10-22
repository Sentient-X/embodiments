namespace RealityLog.OVR
{
    // Names and values remain compatible with existing serialized PoseLogger fields.
    public enum PoseStateMode { Immediate, Raw }

    /// <summary>Samples the selected Meta pose stream, suppressing repeated native timestamps.</summary>
    public sealed class QuestPoseSource
    {
        private double latestTimestamp;
        public void Reset() => latestTimestamp = 0;

        public bool TryRead(OVRPlugin.Node node, PoseStateMode mode, out OVRPlugin.PoseStatef state)
        {
            state = mode switch
            {
                PoseStateMode.Immediate => OVRPlugin.GetNodePoseStateImmediate(node),
                PoseStateMode.Raw => OVRPlugin.GetNodePoseStateRaw(node, OVRPlugin.Step.Render),
                _ => OVRPlugin.PoseStatef.identity,
            };
            if (state.Time <= latestTimestamp) return false;
            latestTimestamp = state.Time;
            return true;
        }
    }
}
