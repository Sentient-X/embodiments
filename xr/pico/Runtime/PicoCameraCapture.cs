using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.ExceptionServices;
using System.Threading;
using System.Threading.Tasks;
using ByteDance.PICO.XR;
using UnityEngine;

namespace SentientCapture.Pico
{
    /// <summary>Borrowed packed RGBA bytes: the consumer must finish before returning.</summary>
    public sealed class PicoCameraFrame
    {
        public XrCameraIdPICO Camera;
        public int Width, Height;
        public byte[] Rgba;
        public long CaptureTimeXrNanoseconds, HostBeforeTicks, HostAfterTicks;
        public ulong ImageId;
    }

    /// <summary>One stereo camera owner. All public calls belong to the Unity main thread.</summary>
    public sealed class PicoCameraCapture : IDisposable
    {
        private const int MaximumDimension = 4096;
        private static PicoCameraCapture activeOwner;
        private static long nextOwnerToken;
        private readonly ulong ownerToken = (ulong)Interlocked.Increment(ref nextOwnerToken);
        private bool cleanupFailed, creationPending;
        private readonly Eye[] eyes = {
            new Eye(XrCameraIdPICO.XR_CAMERA_ID_RGB_LEFT_PICO),
            new Eye(XrCameraIdPICO.XR_CAMERA_ID_RGB_RIGHT_PICO)
        };
        private CancellationTokenSource lifetime;
        private Task<Exception> startupTask;
        private bool starting, running, disposed, attemptedStart;
        public string CalibrationJson { get; private set; }
        public string PollStatusJson => JsonUtility.ToJson(new PollStatus {
            hostTicks = Stopwatch.GetTimestamp(), leftResult = (int)eyes[0].lastResult,
            rightResult = (int)eyes[1].lastResult
        });

        private sealed class Eye
        {
            public readonly XrCameraIdPICO id;
            public bool device, session, capturing;
            public long lastCapture, lastDelivery;
            public PxrResult lastResult = PxrResult.Unknown;
            public PicoCameraFrame frame;
            public Eye(XrCameraIdPICO id) { this.id = id; }
        }
        [Serializable] private sealed class PollStatus
        {
            public long hostTicks;
            public int leftResult, rightResult;
        }
        [Serializable] private sealed class Calibration
        {
            public string cameraClock = "OpenXR XrTime nanoseconds; epoch not mapped";
            public string hostClock = "System.Diagnostics.Stopwatch ticks; read intervals are not a clock mapping";
            public long hostFrequency = Stopwatch.Frequency;
            public string extrinsicsConvention = "PICO SDK GetCameraExtrinsics: relative to XR device, SDK Unity handedness conversion";
            public int width, height, fps = 30;
            public EyeCalibration[] cameras;
        }
        [Serializable] private sealed class EyeCalibration
        {
            public int cameraId;
            public float fx, fy, cx, cy, fovX, fovY;
            public Vector3 position;
            public Quaternion rotation;
        }

        // Choose the smallest actual shared format to bound CPU copying/encoding work.
        // Actual dimensions are recorded; this is no collection-quality claim.
        public static Vector2Int SelectResolution()
        {
            Require(PXR_CameraImage.GetCameraImageResolutionCapability(
                XrCameraIdPICO.XR_CAMERA_ID_RGB_LEFT_PICO, out var left), "left resolutions");
            Require(PXR_CameraImage.GetCameraImageResolutionCapability(
                XrCameraIdPICO.XR_CAMERA_ID_RGB_RIGHT_PICO, out var right), "right resolutions");
            var selected = Vector2Int.zero;
            if (left != null && right != null)
                foreach (var candidate in left)
                    if (candidate.width > 0 && candidate.height > 0 && candidate.width <= MaximumDimension && candidate.height <= MaximumDimension &&
                        candidate.width % 2 == 0 && candidate.height % 2 == 0 &&
                        (selected == Vector2Int.zero || candidate.width * candidate.height < selected.x * selected.y) &&
                        Array.Exists(right, r => r.width == candidate.width && r.height == candidate.height))
                        selected = new Vector2Int(candidate.width, candidate.height);
            if (selected == Vector2Int.zero)
                throw new InvalidOperationException("No shared even PICO stereo resolution within 4096x4096.");
            return selected;
        }

        public async Task StartAsync(int width, int height, CancellationToken token)
        {
            if (attemptedStart || starting || running || disposed) throw new InvalidOperationException("Camera owner cannot start twice.");
            if (width <= 0 || height <= 0 || width > MaximumDimension || height > MaximumDimension)
                throw new ArgumentOutOfRangeException(nameof(width), "Camera dimensions must be between 1 and 4096.");
            if (Interlocked.CompareExchange(ref activeOwner, this, null) != null)
                throw new InvalidOperationException("Previous PICO camera owner has not finished cleanup; wait or relaunch after a cleanup failure.");
            attemptedStart = true;
            starting = true;
            lifetime = CancellationTokenSource.CreateLinkedTokenSource(token);
            var cancelled = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            using (lifetime.Token.Register(() => cancelled.TrySetResult(true)))
            using (var deadline = new CancellationTokenSource())
            {
                // This task owns all native creation and cleanup even after the caller stops
                // waiting. It returns its exception as data, so a late failure is observed.
                Task<Exception> startup = startupTask = StartOwnedAsync(width, height);
                Task winner = await Task.WhenAny(startup, cancelled.Task, Task.Delay(30000, deadline.Token));
                deadline.Cancel();
                if (winner == startup)
                {
                    Exception error = await startup;
                    if (error != null) ExceptionDispatchInfo.Capture(error).Throw();
                    token.ThrowIfCancellationRequested();
                    return;
                }
                lifetime?.Cancel();
                if (winner == cancelled.Task) throw new OperationCanceledException("PICO camera startup cancelled; native cleanup is still draining.", token);
                throw new TimeoutException("PICO camera startup exceeded 30 seconds; native cleanup is still draining. Restart is blocked until it finishes.");
            }
        }

        private async Task<Exception> StartOwnedAsync(int width, int height)
        {
            try
            {
                lifetime.Token.ThrowIfCancellationRequested();
                Require(PXR_CameraImage.GetAvailableCameras(out var available), "enumerate cameras");
                var calibration = new Calibration { width = width, height = height, cameras = new EyeCalibration[eyes.Length] };
                for (int i = 0; i < eyes.Length; i++)
                {
                    var eye = eyes[i];
                    lifetime.Token.ThrowIfCancellationRequested();
                    if (available == null || Array.IndexOf(available, eye.id) < 0)
                        throw new InvalidOperationException($"Required PICO camera unavailable: {eye.id}");
                    ValidateCapabilities(eye.id, width, height);
                    creationPending = true;
                    // The vendor cancellation path destroys resources without checking cleanup
                    // results. Never enter it: finish creation, own the handle, then cancel here.
                    Require(await PXR_CameraImage.CreateCameraDeviceAsync(eye.id, ownerToken, CancellationToken.None), $"create {eye.id}");
                    eye.device = true;
                    creationPending = false;
                    lifetime.Token.ThrowIfCancellationRequested();
                    creationPending = true;
                    Require(await PXR_CameraImage.CreateCameraCaptureSessionAsync(eye.id, ownerToken, width, height,
                        XrCameraImageFpsPICO.XR_CAMERA_IMAGE_FPS_30_PICO,
                        XrCameraImageFormatPICO.XR_CAMERA_IMAGE_FORMAT_RGBA_8888_PICO,
                        XrCameraDataTransferTypePICO.XR_CAMERA_DATA_TRANSFER_TYPE_RAW_BUFFER_PICO,
                        XrCameraModelPICO.XR_CAMERA_MODEL_PINHOLE_PICO, CancellationToken.None), $"session {eye.id}");
                    eye.session = true;
                    creationPending = false;
                    lifetime.Token.ThrowIfCancellationRequested();
                    calibration.cameras[i] = ReadCalibration(eye.id);
                    eye.frame = new PicoCameraFrame { Camera = eye.id, Width = width, Height = height,
                        Rgba = new byte[checked(width * height * 4)] };
                }
                foreach (var eye in eyes)
                {
                    lifetime.Token.ThrowIfCancellationRequested();
                    Require(PXR_CameraImage.BeginCameraCapture(eye.id), $"begin {eye.id}");
                    eye.capturing = true;
                    eye.lastDelivery = Stopwatch.GetTimestamp();
                }
                CalibrationJson = JsonUtility.ToJson(calibration);
                running = true;
                return null;
            }
            catch (Exception error)
            {
                // A failed SDK future may hide an allocated native resource. Hold the owner
                // until process restart when creation ownership could not be established.
                if (creationPending) cleanupFailed = true;
                Cleanup();
                return error;
            }
            finally
            {
                starting = false;
                if (disposed) Cleanup();
            }
        }

        public void Poll(Action<PicoCameraFrame> consume)
        {
            if (!running || disposed) throw new InvalidOperationException("PICO cameras are not running.");
            if (consume == null) throw new ArgumentNullException(nameof(consume));
            try
            {
                foreach (var eye in eyes)
                {
                    long before = Stopwatch.GetTimestamp();
                    eye.lastResult = PXR_CameraImage.AcquireCameraImage(eye.id, eye.lastCapture, out var image, out var time);
                    long after = Stopwatch.GetTimestamp();
                    if (eye.lastResult != PxrResult.SUCCESS)
                    {
                        // Positive OpenXR results carry no image. Never release or repeat an old image.
                        if ((int)eye.lastResult < 0) Require(eye.lastResult, $"acquire {eye.id}");
                        if ((after - eye.lastDelivery) / (double)Stopwatch.Frequency > 3)
                            throw new TimeoutException($"No new {eye.id} image for 3 seconds; result={(int)eye.lastResult}");
                        continue;
                    }
                    try
                    {
                        if (time <= eye.lastCapture)
                            throw new InvalidOperationException($"Non-increasing PICO capture time for {eye.id}.");
                        Require(PXR_CameraImage.GetCameraImageData(eye.id, image, out var data), $"read {eye.id}");
                        CopyPackedRgba(data, eye.frame.Rgba, eye.frame.Width, eye.frame.Height);
                        eye.frame.ImageId = image;
                        eye.frame.CaptureTimeXrNanoseconds = time;
                        eye.frame.HostBeforeTicks = before;
                        eye.frame.HostAfterTicks = after;
                        consume(eye.frame);
                        eye.lastCapture = time;
                        eye.lastDelivery = after;
                    }
                    finally
                    {
                        Require(PXR_CameraImage.ReleaseCameraImage(eye.id, image), $"release {eye.id}");
                    }
                }
            }
            catch
            {
                running = false;
                Cleanup();
                throw;
            }
        }

        internal static void CopyPackedRgba(XrCameraImageDataRawBuffer data, byte[] target, int width, int height)
        {
            if (width <= 0 || height <= 0 || width > MaximumDimension || height > MaximumDimension || target == null)
                throw new InvalidOperationException("Invalid PICO image dimensions or destination.");
            long rowBytes = checked((long)width * 4);
            long required = checked(((long)height - 1) * data.stride + rowBytes);
            if (data.buffer == IntPtr.Zero || data.width != width || data.height != height ||
                data.bytesPerPixel != 4 || data.pixelStride != 4 || data.stride < rowBytes ||
                required > data.bufferSize || required > int.MaxValue || target.LongLength != rowBytes * height)
                throw new InvalidOperationException("Invalid or unsupported PICO RGBA buffer geometry.");
            for (int row = 0; row < height; row++)
                Marshal.Copy(IntPtr.Add(data.buffer, checked((int)(row * (long)data.stride))),
                    target, checked(row * (int)rowBytes), (int)rowBytes);
        }

        public void Dispose()
        {
            if (disposed) return;
            disposed = true;
            running = false;
            lifetime?.Cancel();
            // SDK async creation drains its native future before cancellation completes. Its
            // continuation owns cleanup; destroying here would race that completion.
            if (!starting)
            {
                Cleanup();
                if (cleanupFailed) throw new InvalidOperationException("PICO camera cleanup failed; relaunch before recording again.");
            }
        }

        /// <summary>
        /// Drain the owned SDK future before a combined recording may release its lifecycle.
        /// Must be awaited on the Unity thread; a caller deadline never destroys a pending handle.
        /// </summary>
        public async Task CloseAsync()
        {
            Exception disposalError = null;
            try { Dispose(); }
            catch (Exception error) { disposalError = error; }
            if (startupTask != null) await startupTask;
            if (cleanupFailed)
                throw new InvalidOperationException("PICO camera cleanup is uncertain; relaunch before recording again.", disposalError);
            if (disposalError != null) ExceptionDispatchInfo.Capture(disposalError).Throw();
        }

        private void Cleanup()
        {
            for (int i = eyes.Length - 1; i >= 0; i--)
            {
                var eye = eyes[i];
                if (eye.capturing) { CleanupCall(() => PXR_CameraImage.EndCameraCapture(eye.id), "end", eye.id); eye.capturing = false; }
                if (eye.session) { CleanupCall(() => PXR_CameraImage.DestroyCameraCaptureSession(eye.id, ownerToken), "destroy session", eye.id); eye.session = false; }
                if (eye.device) { CleanupCall(() => PXR_CameraImage.DestroyCameraDevice(eye.id, ownerToken), "destroy device", eye.id); eye.device = false; }
                eye.frame = null;
            }
            lifetime?.Dispose();
            lifetime = null;
            if (!cleanupFailed) Interlocked.CompareExchange(ref activeOwner, null, this);
        }
        private void CleanupCall(Func<PxrResult> action, string operation, XrCameraIdPICO camera)
        {
            try { Require(action(), $"{operation} {camera}"); }
            catch (Exception error) { cleanupFailed = true; UnityEngine.Debug.LogError($"PICO cleanup failed: {error.Message}"); }
        }
        private static void Require(PxrResult result, string operation)
        {
            if (result != PxrResult.SUCCESS)
                throw new InvalidOperationException($"PICO {operation} failed: {result} ({(int)result}).");
        }
        private static void ValidateCapabilities(XrCameraIdPICO id, int width, int height)
        {
            Require(PXR_CameraImage.GetCameraImageResolutionCapability(id, out var resolutions), "resolutions");
            Require(PXR_CameraImage.GetCameraImageFpsCapability(id, out var rates), "rates");
            Require(PXR_CameraImage.GetCameraImageFormatCapability(id, out var formats), "formats");
            Require(PXR_CameraImage.GetCameraDataTransferTypeCapability(id, out var transfers), "transfer types");
            Require(PXR_CameraImage.GetCameraCameraModelCapability(id, out var models), "camera models");
            if (resolutions == null || !Array.Exists(resolutions, r => r.width == width && r.height == height) ||
                rates == null || Array.IndexOf(rates, XrCameraImageFpsPICO.XR_CAMERA_IMAGE_FPS_30_PICO) < 0 ||
                formats == null || Array.IndexOf(formats, XrCameraImageFormatPICO.XR_CAMERA_IMAGE_FORMAT_RGBA_8888_PICO) < 0 ||
                transfers == null || Array.IndexOf(transfers, XrCameraDataTransferTypePICO.XR_CAMERA_DATA_TRANSFER_TYPE_RAW_BUFFER_PICO) < 0 ||
                models == null || Array.IndexOf(models, XrCameraModelPICO.XR_CAMERA_MODEL_PINHOLE_PICO) < 0)
                throw new InvalidOperationException($"Required {width}x{height} 30fps RGBA pinhole capability absent for {id}.");
        }
        private static EyeCalibration ReadCalibration(XrCameraIdPICO id)
        {
            Require(PXR_CameraImage.GetCameraIntrinsics(id, out var intrinsics), "intrinsics");
            Require(PXR_CameraImage.GetCameraExtrinsics(id, out var extrinsics), "extrinsics");
            float[] values = { intrinsics.focalLength.X, intrinsics.focalLength.Y,
                intrinsics.principalPoint.X, intrinsics.principalPoint.Y, intrinsics.fov.X, intrinsics.fov.Y,
                extrinsics.pose.Position.X, extrinsics.pose.Position.Y, extrinsics.pose.Position.Z,
                extrinsics.pose.Orientation.X, extrinsics.pose.Orientation.Y, extrinsics.pose.Orientation.Z, extrinsics.pose.Orientation.W };
            if (Array.Exists(values, value => float.IsNaN(value) || float.IsInfinity(value)) ||
                intrinsics.focalLength.X <= 0 || intrinsics.focalLength.Y <= 0 ||
                (extrinsics.pose.Orientation.X == 0 && extrinsics.pose.Orientation.Y == 0 &&
                 extrinsics.pose.Orientation.Z == 0 && extrinsics.pose.Orientation.W == 0))
                throw new InvalidOperationException($"Invalid PICO camera calibration for {id}.");
            return new EyeCalibration { cameraId = (int)id, fx = intrinsics.focalLength.X, fy = intrinsics.focalLength.Y,
                cx = intrinsics.principalPoint.X, cy = intrinsics.principalPoint.Y,
                fovX = intrinsics.fov.X, fovY = intrinsics.fov.Y,
                position = new Vector3(extrinsics.pose.Position.X, extrinsics.pose.Position.Y, extrinsics.pose.Position.Z),
                rotation = new Quaternion(extrinsics.pose.Orientation.X, extrinsics.pose.Orientation.Y, extrinsics.pose.Orientation.Z, extrinsics.pose.Orientation.W) };
        }
    }
}
