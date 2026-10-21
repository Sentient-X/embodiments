using UnityEngine;
using UnityEngine.XR;

namespace SentientEmbodiments.XR
{
    /// <summary>Observed controller features, preserving unavailable versus released.</summary>
    public static class UnityXrControls
    {
        public readonly struct Button
        {
            public readonly bool Available, Pressed;
            public Button(bool available, bool pressed) { Available = available; Pressed = available && pressed; }
        }
        public static Button Read(XRNode node, InputFeatureUsage<bool> feature)
        {
            var device = InputDevices.GetDeviceAtXRNode(node);
            bool value = false;
            return new Button(device.isValid && device.TryGetFeatureValue(feature, out value), value);
        }
        public static bool TryAxis(XRNode node, out Vector2 axis)
        {
            axis = Vector2.zero;
            var device = InputDevices.GetDeviceAtXRNode(node);
            return device.isValid && device.TryGetFeatureValue(CommonUsages.primary2DAxis, out axis);
        }
    }
}
