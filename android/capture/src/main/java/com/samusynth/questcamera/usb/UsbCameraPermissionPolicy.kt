package com.samusynth.questcamera.usb

/** The caller names its platform; USB permission remains an Android grant per device. */
internal enum class UsbCameraPermissionPolicy {
    HORIZON,
    ANDROID;

    fun allowsVideoRequest(cameraGranted: Boolean, horizonUsbGranted: Boolean): Boolean =
        cameraGranted && (this == ANDROID || horizonUsbGranted)

    companion object {
        fun parse(name: String): UsbCameraPermissionPolicy = when (name) {
            "horizon" -> HORIZON
            "android" -> ANDROID
            else -> throw IllegalArgumentException("Unknown USB camera permission policy: $name")
        }
    }
}
