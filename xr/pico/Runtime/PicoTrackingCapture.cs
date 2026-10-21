using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.XR;

namespace SentientCapture.Pico
{
    /// <summary>Observed Unity XR poses, never relabelled as camera-time poses.</summary>
    public sealed class PicoTrackingCapture : IDisposable
    {
        private bool running;
        private XRInputSubsystem originSource;
        private readonly List<XRInputSubsystem> inputs = new List<XRInputSubsystem>();
        private readonly Observation observation = new Observation();

        [Serializable] private sealed class Observation
        {
            public string poseSource = "Unity XR InputDevices; runtime sampled pose, native sample time unavailable";
            public string coordinateConvention = "Unity XR tracking origin, metres, left handed, quaternion xyzw";
            public string bodyUnavailableReason = "no_motion_trackers";
            public string clock = "System.Diagnostics.Stopwatch ticks; read interval, not pose sample time";
            public long clockFrequency = Stopwatch.Frequency;
            public long hostBeforeTicks, hostAfterTicks;
            public bool originAvailable;
            public int trackingOriginMode;
            public long originChangeSequence, originChangeHostTicks;
            public string originChangeReason = "none";
            public PoseObservation head = new PoseObservation();
            public PoseObservation leftController = new PoseObservation();
            public PoseObservation rightController = new PoseObservation();
        }
        [Serializable] private sealed class PoseObservation
        {
            public bool deviceValid, trackedAvailable, tracked, trackingStateAvailable;
            public int trackingState;
            public bool positionAvailable, rotationAvailable, velocityAvailable, angularVelocityAvailable;
            public Vector3 position, velocity, angularVelocity;
            public Quaternion rotation;
        }

        public void Start()
        {
            if (running) throw new InvalidOperationException("Tracking already running.");
            running = true;
            ReadOrigin();
        }

        public string ReadJson()
        {
            if (!running) throw new InvalidOperationException("Tracking is not running.");
            observation.hostBeforeTicks = Stopwatch.GetTimestamp();
            ReadOrigin();
            ReadPose(XRNode.CenterEye, observation.head);
            ReadPose(XRNode.LeftHand, observation.leftController);
            ReadPose(XRNode.RightHand, observation.rightController);
            observation.hostAfterTicks = Stopwatch.GetTimestamp();
            return JsonUtility.ToJson(observation);
        }

        private void ReadOrigin()
        {
            inputs.Clear();
            SubsystemManager.GetSubsystems(inputs);
            XRInputSubsystem selected = null;
            int count = 0;
            foreach (var input in inputs)
                if (input.running) { selected = input; count++; }
            if (count != 1) selected = null;
            if (selected != originSource)
            {
                if (originSource != null) originSource.trackingOriginUpdated -= OnOriginUpdated;
                originSource = selected;
                if (originSource != null) originSource.trackingOriginUpdated += OnOriginUpdated;
                observation.originChangeReason = "origin_source_changed";
                observation.originChangeSequence++;
                observation.originChangeHostTicks = Stopwatch.GetTimestamp();
            }
            observation.trackingOriginMode = originSource == null ? 0 : (int)originSource.GetTrackingOriginMode();
            observation.originAvailable = originSource != null &&
                observation.trackingOriginMode != (int)TrackingOriginModeFlags.Unknown;
        }

        private void OnOriginUpdated(XRInputSubsystem input)
        {
            observation.originChangeReason = "tracking_origin_updated";
            observation.originChangeSequence++;
            observation.originChangeHostTicks = Stopwatch.GetTimestamp();
        }

        private static void ReadPose(XRNode node, PoseObservation result)
        {
            var device = InputDevices.GetDeviceAtXRNode(node);
            result.deviceValid = device.isValid;
            result.trackedAvailable = device.TryGetFeatureValue(CommonUsages.isTracked, out result.tracked);
            result.trackingStateAvailable = device.TryGetFeatureValue(CommonUsages.trackingState, out var state);
            result.trackingState = (int)state;
            result.positionAvailable = device.TryGetFeatureValue(CommonUsages.devicePosition, out result.position);
            result.rotationAvailable = device.TryGetFeatureValue(CommonUsages.deviceRotation, out result.rotation);
            result.velocityAvailable = device.TryGetFeatureValue(CommonUsages.deviceVelocity, out result.velocity);
            result.angularVelocityAvailable = device.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out result.angularVelocity);
        }

        public void Dispose()
        {
            running = false;
            if (originSource != null) originSource.trackingOriginUpdated -= OnOriginUpdated;
            originSource = null;
        }
    }
}
