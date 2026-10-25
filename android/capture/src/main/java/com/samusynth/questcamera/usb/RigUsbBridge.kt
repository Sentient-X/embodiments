package com.samusynth.questcamera.usb

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.os.Build
import android.util.Log
import android.hardware.usb.UsbConstants
import com.samusynth.questcamera.helper.CameraPermissionRequestActivity

/**
 * Opens the rig's USB devices — the UVC wrist cameras and the ESP32-C6 gripper encoders —
 * and holds their [UsbDeviceConnection]s, so Unity can hand each file descriptor to
 * `sx_rig_attach_usb` (libsx_rig.so). Android does not let an app open `/dev/bus/usb`
 * itself; the fd must come from [UsbManager.openDevice].
 *
 * Unity polls [refresh] and diffs the returned snapshot against what it has attached:
 *  - a new entry is attached to the rig;
 *  - an entry that vanished is detached from the rig, and only then [release]d here.
 * A connection's fd therefore stays open until the native side has let go of it (the ABI
 * leaves fd ownership with Kotlin until `sx_rig_detach_usb`).
 *
 * Permission: the manifest's `USB_DEVICE_ATTACHED` filter with `@xml/device_filter` makes
 * the app the device's default handler once "always open" is chosen, after which plugging a
 * device grants permission with no prompt. A device present without permission is asked for
 * once per attachment ([UsbManager.requestPermission]) and never re-prompted in a loop.
 *
 * Every broadcast only triggers a re-enumeration from [UsbManager]; intent extras are never
 * trusted, so a spoofed broadcast cannot attach or drop a device.
 */
class RigUsbBridge @JvmOverloads constructor(
    context: Context,
    permissionPolicy: String = "horizon",
) : AutoCloseable {
    private val cameraPolicy = UsbCameraPermissionPolicy.parse(permissionPolicy)
    private val appContext = context.applicationContext
    private val usbManager = appContext.getSystemService(Context.USB_SERVICE) as UsbManager
    private val lock = Any()

    private class Held(val device: UsbDevice, val connection: UsbDeviceConnection, val serial: String?)

    /** Open connections of devices still attached, by attachment key. */
    private val open = HashMap<String, Held>()
    /** Open connections whose device is gone, awaiting [release] after the native detach. */
    private val gone = HashMap<String, Held>()
    /** Attachment keys already asked for permission. */
    private val permissionRequested = HashSet<String>()
    private var lastError: String? = null
    private var closed = false

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            runCatching { refresh() }.onFailure { Log.e(TAG, "USB refresh after ${intent?.action} failed", it) }
        }
    }

    init {
        val filter = IntentFilter().apply {
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
            addAction(ACTION_USB_PERMISSION)
        }
        // Not ContextCompat.registerReceiver: it needs androidx.core 1.9, and the Unity app
        // resolves an older core (appcompat 1.5.1), where the call is a NoSuchMethodError that
        // left the bridge unconstructed on the 3S. The platform overload with flags exists from
        // API 26; RECEIVER_EXPORTED is required from API 33 for a receiver that also hears
        // system broadcasts.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            appContext.registerReceiver(receiver, filter, Context.RECEIVER_EXPORTED)
        } else {
            @Suppress("UnspecifiedRegisterReceiverFlag")
            appContext.registerReceiver(receiver, filter)
        }
    }

    /**
     * Re-enumerates, opens every permitted rig device not yet open, asks permission once for
     * the rest, moves connections of unplugged devices to the pending-release set, and
     * returns the snapshot JSON (see [snapshotJson]).
     */
    fun refresh(): String = synchronized(lock) {
        if (closed) return@synchronized snapshotJsonLocked(emptyList())
        val present = usbManager.deviceList.values.filter { isRigDevice(it.vendorId, it.productId) }
        val presentKeys = present.map(::keyOf).toSet()

        for (key in open.keys.toList()) {
            if (key !in presentKeys) gone[key] = open.remove(key)!!
        }
        permissionRequested.retainAll(presentKeys)

        for (device in present) {
            val key = keyOf(device)
            if (key in open) continue
            if (!usbManager.hasPermission(device)) {
                // Horizon OS silently denies a video-class device's permission without CAMERA +
                // USB_CAMERA; asking before they are granted would spend the one request.
                val blocked = isVideoDevice(device) &&
                    !hasUsbCameraPermissions()
                if (!blocked && permissionRequested.add(key)) requestPermission(device)
                continue
            }
            val connection = usbManager.openDevice(device)
            if (connection == null) {
                lastError = "openDevice returned null for ${device.deviceName}"
                Log.e(TAG, lastError!!)
                continue
            }
            // Read once at open: getSerial() is a descriptor read on the bus.
            open[key] = Held(device, connection, runCatching { connection.serial }.getOrNull())
            Log.i(TAG, "Opened ${describe(device)} fd=${connection.fileDescriptor}")
        }
        snapshotJsonLocked(present)
    }

    /**
     * The current snapshot without re-enumerating:
     * `{"devices":[{"key","deviceName","vendorId","productId","fd","serial"}],
     *   "present":N, "awaitingPermission":[deviceName...], "pendingRelease":[key...], "error":...}`.
     * `present` counts rig devices on the bus, permitted or not.
     */
    fun snapshotJson(): String = synchronized(lock) {
        snapshotJsonLocked(usbManager.deviceList.values.filter { isRigDevice(it.vendorId, it.productId) })
    }

    /** Closes the connection behind [key] once the rig has detached it. Idempotent. */
    fun release(key: String) = synchronized(lock) {
        val held = gone.remove(key) ?: open.remove(key) ?: return@synchronized
        runCatching { held.connection.close() }
        Log.i(TAG, "Released ${describe(held.device)}")
    }

    override fun close() = synchronized(lock) {
        if (closed) return@synchronized
        closed = true
        runCatching { appContext.unregisterReceiver(receiver) }
        (open.values + gone.values).forEach { runCatching { it.connection.close() } }
        open.clear()
        gone.clear()
    }

    private fun hasUsbCameraPermissions(): Boolean = cameraPolicy.allowsVideoRequest(
        appContext.checkSelfPermission(android.Manifest.permission.CAMERA) == android.content.pm.PackageManager.PERMISSION_GRANTED,
        appContext.checkSelfPermission(CameraPermissionRequestActivity.USB_CAMERA_PERMISSION) == android.content.pm.PackageManager.PERMISSION_GRANTED,
    )

    private fun requestPermission(device: UsbDevice) {
        val intent = Intent(ACTION_USB_PERMISSION).setPackage(appContext.packageName)
        // The system fills the device and grant extras in, so the PendingIntent is mutable.
        val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
        val pending = PendingIntent.getBroadcast(appContext, 0, intent, flags)
        usbManager.requestPermission(device, pending)
        Log.i(TAG, "Requested permission for ${describe(device)}")
    }

    private fun snapshotJsonLocked(present: List<UsbDevice>): String {
        val awaiting = present.filter { keyOf(it) !in open }.map { it.deviceName }
        val devices = open.entries.joinToString(",", "[", "]") { (key, held) ->
            "{\"key\":${quote(key)},\"deviceName\":${quote(held.device.deviceName)}," +
                "\"vendorId\":${held.device.vendorId},\"productId\":${held.device.productId}," +
                "\"fd\":${held.connection.fileDescriptor}," +
                "\"serial\":${held.serial?.let(::quote) ?: "null"}}"
        }
        return "{\"devices\":$devices,\"present\":${present.size}," +
            "\"usbCameraPermission\":${hasUsbCameraPermissions()}," +
            "\"awaitingPermission\":${awaiting.joinToString(",", "[", "]") { quote(it) }}," +
            "\"pendingRelease\":${gone.keys.joinToString(",", "[", "]") { quote(it) }}," +
            "\"error\":${lastError?.let(::quote) ?: "null"}}"
    }

    companion object {
        private const val TAG = "RigUsbBridge"
        const val ACTION_USB_PERMISSION = "com.samusynth.questcamera.usb.USB_PERMISSION"

        /** UVC wrist camera. */
        const val WRIST_CAMERA_VENDOR_ID = 0x32E4
        const val WRIST_CAMERA_PRODUCT_ID = 0x9230
        /** Espressif (ESP32-C6 gripper encoders, any product id). */
        const val ESPRESSIF_VENDOR_ID = 0x303A

        @JvmStatic
        fun isRigDevice(vendorId: Int, productId: Int): Boolean =
            (vendorId == WRIST_CAMERA_VENDOR_ID && productId == WRIST_CAMERA_PRODUCT_ID) ||
                vendorId == ESPRESSIF_VENDOR_ID

        /** A device with a video-class interface (a UVC camera). */
        private fun isVideoDevice(device: UsbDevice): Boolean =
            device.deviceClass == UsbConstants.USB_CLASS_VIDEO ||
                (0 until device.interfaceCount).any {
                    device.getInterface(it).interfaceClass == UsbConstants.USB_CLASS_VIDEO
                }

        /**
         * One attachment: the bus path plus Android's per-attachment device id, so a device
         * replugged at the same path is a new attachment, never the old one reopened.
         */
        private fun keyOf(device: UsbDevice): String = "${device.deviceName}#${device.deviceId}"

        private fun describe(device: UsbDevice): String =
            "%04x:%04x at %s".format(device.vendorId, device.productId, device.deviceName)

        private fun quote(value: String): String =
            "\"" + value.replace("\\", "\\\\").replace("\"", "\\\"") + "\""
    }
}
