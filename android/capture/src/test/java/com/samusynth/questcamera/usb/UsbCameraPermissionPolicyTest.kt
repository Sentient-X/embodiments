package com.samusynth.questcamera.usb

import org.junit.Assert.*
import org.junit.Test

class UsbCameraPermissionPolicyTest {
    @Test fun horizonStillRequiresBothGrantsAndAndroidNeverRequiresHorizonGrant() {
        for (camera in listOf(false, true)) for (usb in listOf(false, true)) {
            assertEquals(camera && usb, UsbCameraPermissionPolicy.parse("horizon").allowsVideoRequest(camera, usb))
            assertEquals(camera, UsbCameraPermissionPolicy.parse("android").allowsVideoRequest(camera, usb))
        }
    }

    @Test(expected = IllegalArgumentException::class)
    fun anUnknownPolicyNeverSilentlyPermitsUsbVideo() {
        UsbCameraPermissionPolicy.parse("pico-maybe")
    }
}
