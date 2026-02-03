package com.oculus.camerademo

import android.annotation.SuppressLint
import android.app.Application
import android.content.Context
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.hardware.camera2.*
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.Surface
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.AndroidViewModel
import androidx.core.content.ContextCompat
import android.Manifest
import android.content.pm.PackageManager
import java.util.concurrent.Executor
import kotlinx.serialization.Serializable

/**
 * =============================================================================
 * CameraDemoViewModel.kt
 *
 * Author: Layla Ashry
 *
 * Description:
 * ------------
 * Android ViewModel responsible for discovering Meta Quest cameras, extracting
 * calibration data, managing camera lifecycles, encoding video streams, and
 * transmitting encoded frames with metadata to a remote ROS system.
 *
 * Data Flow:
 * ----------
 * Meta Quest Camera → Camera2 API → MediaCodec (H.264)
 *                  → Streamer (network transport)
 *                  → ROS receiver
 * =============================================================================
 */

class CameraDemoViewModel(application: Application) : AndroidViewModel(application) {

    companion object {
        private const val TAG = "CameraDemoVM"
        val KEY_POSITION: CameraCharacteristics.Key<Int>? = null
        val KEY_SOURCE: CameraCharacteristics.Key<Int>? = null

    }

    private val _uiState = MutableLiveData(CameraUiState())
    val uiState: LiveData<CameraUiState> = _uiState

    private val _cameraEvents = MutableLiveData<CameraEvent>()
    val cameraEvents: LiveData<CameraEvent> = _cameraEvents

    private val cameraManager = application.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    private val cameraThread = HandlerThread("CameraThread").apply { start() }
    private val cameraHandler = Handler(cameraThread.looper)

    private val encoderThread = HandlerThread("EncoderThread").apply { start() }
    private val encoderHandler = Handler(encoderThread.looper)

    // Holds the state for each active or opening camera stream
    private val cameraStreams = mutableMapOf<String, CameraStreamState>()

    private data class CameraStreamState(
        var cameraDevice: CameraDevice? = null,
        var cameraSession: CameraCaptureSession? = null,
        var activeConfig: CameraConfig? = null, // The config used to open this stream
        var previewSurface: Surface? = null,    // The surface provided by TextureView
        var encoder: MediaCodec? = null,
        var encoderInputSurface: Surface? = null,
        val streamer: Streamer = Streamer(), // Assuming Streamer is a class you have
        var isStreaming: Boolean = false
    )

    fun init() {
        Log.d(TAG, "ViewModel init() called.")
        loadCameraConfigs()
    }

    private fun loadCameraConfigs() {
        try {
            val cameraIds = cameraManager.cameraIdList
            Log.i(TAG, "Found ${cameraIds.size} camera IDs: ${cameraIds.joinToString()}")
            val availableConfigs = mutableListOf<CameraConfig>()

            cameraIds.forEach { id ->
                Log.d(TAG, "Processing Camera ID: $id")
                val characteristics = cameraManager.getCameraCharacteristics(id)
                var chosenIntrinsicsKey: CameraCharacteristics.Key<FloatArray>? = null
                var chosenDistortionKey: CameraCharacteristics.Key<FloatArray>? = null

                // Prioritize standard keys
                if (characteristics.keys.contains(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)) {
                    chosenIntrinsicsKey = CameraCharacteristics.LENS_INTRINSIC_CALIBRATION
                    Log.i(TAG, "For Camera ID $id - Using standard LENS_INTRINSIC_CALIBRATION key.")
                    if (characteristics.keys.contains(CameraCharacteristics.LENS_DISTORTION)) {
                        chosenDistortionKey = CameraCharacteristics.LENS_DISTORTION
                        Log.i(TAG, "For Camera ID $id - Using standard LENS_DISTORTION key.")
                    } else {
                        Log.w(TAG, "For Camera ID $id - Standard LENS_DISTORTION key not found.")
                    }
                } else { // Standard LENS_INTRINSIC_CALIBRATION not found
                    Log.w(TAG, "For Camera ID $id - Standard LENS_INTRINSIC_CALIBRATION key not found. Trying Meta-specific key by name.")
                    val metaIntrinsicsKeyCandidate = characteristics.keys.find { it.name == "com.meta.camera.CALIBRATION_INTRINSICS" }
                    val castedMetaIntrinsicsKey = metaIntrinsicsKeyCandidate as? CameraCharacteristics.Key<FloatArray>

                    if (castedMetaIntrinsicsKey != null) {
                        chosenIntrinsicsKey = castedMetaIntrinsicsKey
                        Log.i(TAG, "For Camera ID $id - Found and using Meta-specific CALIBRATION_INTRINSICS key: ${metaIntrinsicsKeyCandidate?.name}")

                        val metaDistortionKeyCandidate = characteristics.keys.find { it.name == "com.meta.camera.CALIBRATION_DISTORTION" }
                        val castedMetaDistortionKey = metaDistortionKeyCandidate as? CameraCharacteristics.Key<FloatArray>
                        if (castedMetaDistortionKey != null) {
                            chosenDistortionKey = castedMetaDistortionKey
                            Log.i(TAG, "For Camera ID $id - Found and using Meta-specific CALIBRATION_DISTORTION key: ${metaDistortionKeyCandidate?.name}")
                        } else {
                            Log.w(TAG, "For Camera ID $id - Meta-specific CALIBRATION_DISTORTION key by name found, but not of FloatArray type, or not found at all.")
                        }
                    } else {
                        Log.w(TAG, "For Camera ID $id - Meta-specific CALIBRATION_INTRINSICS key by name found, but not of FloatArray type, or not found at all.")
                    }
                }
                Log.d(TAG, "For Camera ID $id - chosenIntrinsicsKey: $chosenIntrinsicsKey")
                Log.d(TAG, "For Camera ID $id - chosenDistortionKey: $chosenDistortionKey")


                // Attempt to find Meta-specific keys by name for POSITION and SOURCE
                val metaPositionKeyCandidate = characteristics.keys.find { it.name == "com.meta.camera.POSITION" }
                val castedMetaPositionKey = metaPositionKeyCandidate as? CameraCharacteristics.Key<Int>
                val actualPositionKey = castedMetaPositionKey ?: KEY_POSITION // Fallback to companion object key
                if (castedMetaPositionKey != null) {
                    Log.i(TAG, "For Camera ID $id - Found and using Meta-specific POSITION key: ${metaPositionKeyCandidate?.name}")
                }

                val metaSourceKeyCandidate = characteristics.keys.find { it.name == "com.meta.camera.SOURCE" }
                val castedMetaSourceKey = metaSourceKeyCandidate as? CameraCharacteristics.Key<Int>
                val actualSourceKey = castedMetaSourceKey ?: KEY_SOURCE // Fallback to companion object key
                if (castedMetaSourceKey != null) {
                    Log.i(TAG, "For Camera ID $id - Found and using Meta-specific SOURCE key: ${metaSourceKeyCandidate?.name}")
                }
                Log.d(TAG, "For Camera ID $id - Using actualPositionKey name: ${actualPositionKey?.name}, actualSourceKey name: ${actualSourceKey?.name}")


                val config = createCameraConfig(id, characteristics, chosenIntrinsicsKey, chosenDistortionKey, actualPositionKey, actualSourceKey)
                if (config != null) {
                    Log.i(TAG, "Successfully created config for camera $id: $config")
                    availableConfigs.add(config)
                } else {
                    Log.w(TAG, "Failed to create config for camera $id. createCameraConfig returned null.")
                }
            }
            _uiState.postValue(_uiState.value?.copy(availableCameras = availableConfigs))
            Log.i(TAG, "loadCameraConfigs FINISHED. Found ${availableConfigs.size} usable camera configs. UIState posted.")
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to access camera manager", e)
            postEvent("Failed to access camera service")
        } catch (e: Exception) { // Catch any other unexpected errors during config loading
            Log.e(TAG, "Unexpected error during loadCameraConfigs", e)
            postEvent("Error loading camera configs")
        }
    }


    private fun createCameraConfig(
        id: String, chars: CameraCharacteristics,
        intrinsicsKeyToUse: CameraCharacteristics.Key<FloatArray>?,
        distortionKeyToUse: CameraCharacteristics.Key<FloatArray>?,
        positionKeyToUse: CameraCharacteristics.Key<Int>?,
        sourceKeyToUse: CameraCharacteristics.Key<Int>?
    ): CameraConfig? {
        Log.d(TAG, "createCameraConfig for ID $id. IntrinsicsKey: $intrinsicsKeyToUse, DistortionKey: $distortionKeyToUse")

        val streamConfigMap = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        if (streamConfigMap == null) {
            Log.e(TAG, "Camera $id: SCALER_STREAM_CONFIGURATION_MAP is null. Cannot create config.")
            return null
        }

        var outputSizes: Array<Size>? = null
        var chosenFormatString: String? = null
        try {
            outputSizes = streamConfigMap.getOutputSizes(SurfaceTexture::class.java)
            if (outputSizes != null && outputSizes.isNotEmpty()) {
                chosenFormatString = "SurfaceTexture"
                Log.i(TAG, "Camera $id: Found ${outputSizes.size} sizes for SurfaceTexture.")
            } else {
                Log.w(TAG, "Camera $id: No output sizes found for SurfaceTexture. OutputSizes was null or empty.")
                outputSizes = null
            }
        } catch (e: Exception) {
            Log.e(TAG, "Camera $id: Exception getting sizes for SurfaceTexture.", e)
            outputSizes = null
        }


        // if SurfaceTexture fails, try YUV_420_888 (common for processing/encoding)
        if (outputSizes == null || outputSizes.isEmpty()) {
            Log.d(TAG, "Camera $id: Attempting ImageFormat.YUV_420_888.")
            try {
                outputSizes = streamConfigMap.getOutputSizes(ImageFormat.YUV_420_888)
                if (outputSizes != null && outputSizes.isNotEmpty()) {
                    chosenFormatString = "ImageFormat.YUV_420_888"
                    Log.i(TAG, "Camera $id: Found ${outputSizes.size} sizes for ImageFormat.YUV_420_888.")
                } else {
                    Log.w(TAG, "Camera $id: No output sizes found for ImageFormat.YUV_420_888. OutputSizes was null or empty.")
                    outputSizes = null
                }
            } catch (e: Exception) {
                Log.e(TAG, "Camera $id: Exception getting sizes for ImageFormat.YUV_420_888.", e)
                outputSizes = null
            }
        }


        if (outputSizes == null || outputSizes.isEmpty()) {
            Log.d(TAG, "Camera $id: Attempting generic Surface.")
            try {
                outputSizes = streamConfigMap.getOutputSizes(Surface::class.java)
                if (outputSizes != null && outputSizes.isNotEmpty()) {
                    chosenFormatString = "Surface (Generic)"
                    Log.i(TAG, "Camera $id: Found ${outputSizes.size} sizes for generic Surface.")
                } else {
                    Log.w(TAG, "Camera $id: No output sizes found for generic Surface. OutputSizes was null or empty.")
                    outputSizes = null
                }
            } catch (e: Exception) {
                Log.e(TAG, "Camera $id: Exception getting sizes for generic Surface.", e)
                outputSizes = null
            }
        }

        if (outputSizes == null || outputSizes.isEmpty()) {
            Log.e(TAG, "Camera $id: No output sizes found for SurfaceTexture, YUV_420_888, or generic Surface. Cannot create config.")
            return null
        }
        Log.d(TAG, "Camera $id: Using sizes from $chosenFormatString. Available output sizes: ${outputSizes.joinToString { "${it.width}x${it.height}" }}")


        val previewSize = outputSizes.firstOrNull { it.width <= 1920 && it.height <= 1080 } // added height constraint
            ?: outputSizes.firstOrNull()

        if (previewSize == null) {
            Log.e(TAG, "Camera $id: Could not determine a suitable previewSize from available $chosenFormatString sizes (all were null or list empty after filter). Cannot create config.")
            return null
        }
        Log.i(TAG, "Camera $id: Selected preview size: ${previewSize.width}x${previewSize.height} from $chosenFormatString")

        var intrinsics: CameraIntrinsics? = null
        var extrinsics: CameraExtrinsics? = null
        var positionString = "Unknown"
        var isPassthrough = false

        try {
            if (intrinsicsKeyToUse != null) {
                val intrinsicsArray = chars.get(intrinsicsKeyToUse)
                intrinsics = intrinsicsArray?.let {
                    val distortionValuesArray = if (distortionKeyToUse != null) chars.get(distortionKeyToUse) else null
                    if (it.size >= 4) { // fx, fy, cx, cy are usually the first 4
                        CameraIntrinsics(it[0], it[1], it[2], it[3], distortionValuesArray)
                    } else {
                        Log.w(TAG, "Camera $id: Intrinsics array too short (${it.size}) to extract fx,fy,cx,cy.")
                        null
                    }
                }
            }
            val poseRotation = chars.get(CameraCharacteristics.LENS_POSE_ROTATION)
            val poseTranslation = chars.get(CameraCharacteristics.LENS_POSE_TRANSLATION)
            if (poseRotation != null && poseTranslation != null) {
                extrinsics = CameraExtrinsics(poseRotation, poseTranslation)
            }
            positionString = when (chars.get(CameraCharacteristics.LENS_FACING)) {
                CameraCharacteristics.LENS_FACING_FRONT -> "Front"
                CameraCharacteristics.LENS_FACING_BACK -> "Back"
                CameraCharacteristics.LENS_FACING_EXTERNAL -> "External"
                else -> "Unknown (Standard)"
            }

            positionKeyToUse?.let { key ->
                chars.get(key)?.let { posVal ->
                    positionString = when(posVal) {
                        0 -> "Left (Meta)"
                        1 -> "Right (Meta)"

                        else -> "Position $posVal (Meta)"
                    }
                }
            }
            sourceKeyToUse?.let { key -> chars.get(key)?.let { srcVal -> isPassthrough = srcVal == 0 } } // Assuming 0 means passthrough for Meta

        } catch (e: Exception) {
            Log.e(TAG, "Error reading detailed camera characteristics for $id (intrinsics, extrinsics, etc.)", e)
        }
        return CameraConfig(id, previewSize.width, previewSize.height, positionString, isPassthrough, intrinsics, extrinsics)
    }


    private fun postEvent(message: String) {
        _cameraEvents.postValue(CameraEvent.NotificationEvent(message))
    }

    private fun hasPermissions(): Boolean {
        val application = getApplication<Application>()
        return ContextCompat.checkSelfPermission(application, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(application, PermissionManager.HZOS_CAMERA_PERMISSION) == PackageManager.PERMISSION_GRANTED
    }

    @SuppressLint("MissingPermission")
    fun startDualCameraStreams(surfaceForCam50: Surface, surfaceForCam51: Surface) {
        Log.d(TAG, "Enter startDualCameraStreams. Surface50: $surfaceForCam50, Surface51: $surfaceForCam51")
        if (!hasPermissions()) {
            postEvent("Camera permissions not granted")
            Log.w(TAG, "startDualCameraStreams: Permissions not granted.")
            return
        }
        val currentUiState = _uiState.value
        val availableConfigs = currentUiState?.availableCameras
        if (availableConfigs == null || availableConfigs.isEmpty()) {
            postEvent("Camera configurations not yet loaded or empty.")
            Log.w(TAG, "startDualCameraStreams: Available camera configurations not loaded or empty. Current list: $availableConfigs")
            return
        }

        val configCam50 = availableConfigs.find { it.id == "50" }
        val configCam51 = availableConfigs.find { it.id == "51" }

        Log.d(TAG, "Config for Cam50: $configCam50, Config for Cam51: $configCam51 from a list of ${availableConfigs.size} configs: $availableConfigs")

        var openedAny = false
        if (configCam50 == null) {
            postEvent("Camera config for ID 50 not found.")
            Log.w(TAG, "startDualCameraStreams: Camera config for ID 50 not found in the loaded list.")
        } else {
            cameraStreams["50"] = CameraStreamState(activeConfig = configCam50, previewSurface = surfaceForCam50)
            Log.d(TAG, "startDualCameraStreams: Setting up Cam50. Config: $configCam50, Surface: ${cameraStreams["50"]?.previewSurface}")
            openCameraInternal("50", surfaceForCam50)
            openedAny = true
        }
        if (configCam51 == null) {
            postEvent("Camera config for ID 51 not found.")
            Log.w(TAG, "startDualCameraStreams: Camera config for ID 51 not found in the loaded list.")
        } else {
            cameraStreams["51"] = CameraStreamState(activeConfig = configCam51, previewSurface = surfaceForCam51)
            Log.d(TAG, "startDualCameraStreams: Setting up Cam51. Config: $configCam51, Surface: ${cameraStreams["51"]?.previewSurface}")
            openCameraInternal("51", surfaceForCam51)
            openedAny = true
        }

        if (!openedAny) {
            postEvent("Failed to start any camera stream as configs were missing.")
        }
        Log.d(TAG, "Exit startDualCameraStreams")
    }

    @SuppressLint("MissingPermission")
    private fun openCameraInternal(cameraId: String, targetSurface: Surface) {
        Log.d(TAG, "Attempting to open camera: $cameraId with surface: $targetSurface")
        val streamState = cameraStreams[cameraId] ?: run {
            Log.e(TAG, "No stream state found for camera $cameraId during open.")
            postEvent("Error: State not found for camera $cameraId")
            return
        }
        streamState.previewSurface = targetSurface // ensure preview surface is set in state
        try {
            cameraManager.openCamera(cameraId, object : CameraDevice.StateCallback() {
                override fun onOpened(device: CameraDevice) {
                    Log.i(TAG, "SUCCESS: Camera $cameraId opened. Device: ${device.id}")
                    streamState.cameraDevice = device
                    createSessionInternal(cameraId, isStreaming = streamState.isStreaming)
                }
                override fun onDisconnected(device: CameraDevice) {
                    Log.w(TAG, "LIFECYCLE: Camera $cameraId disconnected. Device: ${device.id}")
                    cleanupStream(cameraId)
                }
                override fun onError(device: CameraDevice, error: Int) {
                    Log.e(TAG, "ERROR: Error opening camera $cameraId. Device: ${device.id}, ErrorCode: $error")
                    device.close() // Close the problematic device
                    cleanupStream(cameraId) // Then cleanup our state
                    postEvent("Error with camera $cameraId: $error")
                }
            }, cameraHandler)
        } catch (e: Exception) { // Catch SecurityException or others
            Log.e(TAG, "EXCEPTION: Failed to open camera $cameraId", e)
            cleanupStream(cameraId) // Cleanup state on exception
            postEvent("Failed to open camera $cameraId: ${e.message}")
        }
    }

    private fun setupEncoderInternal(cameraId: String): Boolean {
        val streamState = cameraStreams[cameraId] ?: run {
            Log.e(TAG, "No stream state for $cameraId in setupEncoderInternal")
            return false
        }
        val config = streamState.activeConfig ?: run {
            Log.e(TAG, "No active config for $cameraId in setupEncoderInternal")
            return false
        }
        try {
            // Clean up existing encoder resources if any
            streamState.encoder?.stop()
            streamState.encoder?.release()
            streamState.encoder = null
            streamState.encoderInputSurface?.release()
            streamState.encoderInputSurface = null

            val format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, config.width, config.height)
            format.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            format.setInteger(MediaFormat.KEY_BIT_RATE, 5_000_000) // 5 Mbps
            format.setInteger(MediaFormat.KEY_FRAME_RATE, 30)
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1) // 1 second I-frame interval

            val newEncoder = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
            newEncoder.setCallback(object : MediaCodec.Callback() {
                override fun onInputBufferAvailable(codec: MediaCodec, index: Int) {
                }

                override fun onOutputBufferAvailable(codec: MediaCodec, index: Int, info: MediaCodec.BufferInfo) {
                    val currentStreamState = cameraStreams[cameraId]
                    if (currentStreamState?.isStreaming == true) {
                        codec.getOutputBuffer(index)?.let { buffer ->
                            // Create the metadata package for this frame
                            val metadata = currentStreamState.activeConfig?.let { config ->
                                FrameMetadata(
                                    timestamp = System.nanoTime(),
                                    intrinsics = config.intrinsics,
                                    extrinsics = config.extrinsics
                                )
                            }

                            if (metadata != null) {
                                currentStreamState.streamer.queueFrame(buffer, info, metadata)
                            } else {
                                Log.w(TAG, "Could not queue frame for $cameraId because metadata was null")
                            }

                            codec.releaseOutputBuffer(index, false)
                        }
                    } else {
                        try { codec.releaseOutputBuffer(index, false) }
                        catch (_: IllegalStateException) { Log.e(TAG,"Encoder $cameraId already released onOutputBufferAvailable") }
                    }
                }


                override fun onError(codec: MediaCodec, e: MediaCodec.CodecException) {
                    Log.e(TAG, "Encoder error for camera $cameraId: ${e.message}", e)
                }
                override fun onOutputFormatChanged(codec: MediaCodec, format: MediaFormat) {
                    Log.d(TAG, "Encoder output format changed for camera $cameraId: $format")
                }
            }, encoderHandler)

            newEncoder.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            streamState.encoderInputSurface = newEncoder.createInputSurface()
            newEncoder.start()
            streamState.encoder = newEncoder
            Log.i(TAG, "Encoder setup complete for camera $cameraId.")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to setup encoder for camera $cameraId", e)
            streamState.encoder?.release()
            streamState.encoder = null
            streamState.encoderInputSurface?.release()
            streamState.encoderInputSurface = null
            return false
        }
    }

    @SuppressLint("MissingPermission")
    private fun createSessionInternal(cameraId: String, isStreaming: Boolean) {
        val streamState = cameraStreams[cameraId] ?: run {
            Log.e(TAG, "No stream state for $cameraId in createSessionInternal")
            return
        }
        val cameraDevice = streamState.cameraDevice ?: run {
            Log.e(TAG, "CameraDevice null for $cameraId in createSessionInternal. Cannot create session.")
            return
        }
        val previewSurface = streamState.previewSurface ?: run {
            Log.e(TAG, "PreviewSurface null for $cameraId in createSessionInternal. Cannot create session.")
            return
        }
        Log.d(TAG, "Attempting to create session for camera $cameraId. Requested streaming: $isStreaming. Device: ${cameraDevice.id}, PreviewSurface: $previewSurface")

        var effectiveIsStreaming = isStreaming
        if (isStreaming) { // Only setup encoder if we intend to stream
            if (streamState.encoder == null || streamState.encoderInputSurface == null) {
                Log.d(TAG, "Streaming requested for $cameraId, setting up encoder...")
                if (!setupEncoderInternal(cameraId)) {
                    Log.e(TAG, "Failed to setup encoder for $cameraId. Cannot stream.")
                    postEvent("Encoder setup failed for Cam $cameraId")
                    effectiveIsStreaming = false
                }
            }
        }

        try {
            streamState.cameraSession?.close()
            streamState.cameraSession = null

            val targets = mutableListOf<Surface>()
            targets.add(previewSurface)

            if (effectiveIsStreaming && streamState.encoderInputSurface != null) {
                streamState.encoderInputSurface?.let { targets.add(it) }
                Log.i(TAG, "Session for $cameraId will include preview and encoder surfaces: $targets")
            } else {
                Log.i(TAG, "Session for $cameraId will include preview surface ONLY: $targets")
            }

            if (targets.isEmpty()){
                Log.e(TAG, "No surfaces to create a session for $cameraId.")
                return
            }

            val outputConfigurations = targets.map { OutputConfiguration(it) }
            val sessionConfiguration = SessionConfiguration(
                SessionConfiguration.SESSION_REGULAR,
                outputConfigurations,
                executor,
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        Log.i(TAG, "SUCCESS: CaptureSession configured for camera $cameraId. Session: $session")
                        streamState.cameraSession = session
                        try {
                            val builder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
                            targets.forEach { surface -> builder.addTarget(surface) }
                            session.setRepeatingRequest(builder.build(), null, cameraHandler) // Handler for repeating request
                            Log.i(TAG, "SUCCESS: Repeating request set for camera $cameraId.")
                        } catch (e: Exception) { // CameraAccessException or IllegalStateException
                            Log.e(TAG, "ERROR: Error setting repeating request for $cameraId", e)
                            postEvent("Error starting preview for Cam $cameraId")
                        }
                    }
                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        Log.e(TAG, "ERROR: Failed to configure CaptureSession for camera $cameraId. Session: $session")
                        postEvent("Session config failed for Cam $cameraId")
                    }

                    override fun onClosed(session: CameraCaptureSession) {
                        super.onClosed(session)
                        Log.d(TAG, "LIFECYCLE: CaptureSession closed for camera $cameraId. Session: $session")
                        // If the session is closed unexpectedly, we might not need to do anything here
                        // as cleanupStream might be called from other places (onDisconnected, onError).
                    }
                }
            )
            Log.d(TAG, "Calling createCaptureSession for $cameraId...")
            cameraDevice.createCaptureSession(sessionConfiguration)
        } catch (e: Exception) { // CameraAccessException or other
            Log.e(TAG, "EXCEPTION: Error creating capture session for $cameraId", e)
            postEvent("Session creation error for Cam $cameraId")
        }
    }

    fun startStreaming(cameraId: String, ip: String, port: Int) {
        val streamState = cameraStreams[cameraId] ?: run {
            Log.e(TAG, "No stream state for $cameraId in startStreaming.")
            postEvent("Error: Cannot start streaming, state not found for Cam $cameraId")
            return
        }
        if (streamState.isStreaming) {
            Log.w(TAG, "Camera $cameraId is already streaming.")
            return
        }
        // Ensure encoder is ready. If not, setupEncoderInternal will be called by createSessionInternal
        // when isStreaming is true.
        if (streamState.encoder == null || streamState.encoderInputSurface == null) {
            Log.d(TAG, "Encoder not ready for $cameraId, attempting setup via createSessionInternal before streaming.")
            // No need to call setupEncoderInternal directly here if createSessionInternal will handle it
        }

        try {
            streamState.streamer.connect(ip, port) // Assuming this can throw
            streamState.isStreaming = true // Set before createSession to ensure encoder is setup

            // Update UI State
            if (cameraId == "50") {
                _uiState.postValue(_uiState.value?.copy(isStreamingCam50 = true))
            } else if (cameraId == "51") {
                _uiState.postValue(_uiState.value?.copy(isStreamingCam51 = true))
            }

            // Recreate session to include encoder surface
            createSessionInternal(cameraId, isStreaming = true) // This will now try to set up the encoder

            postEvent("Streaming started for Cam $cameraId")
            Log.i(TAG, "Streaming started for Cam $cameraId on $ip:$port")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start streaming for Cam $cameraId (streamer.connect or other)", e)
            streamState.isStreaming = false // Revert on failure
            if (cameraId == "50") {
                _uiState.postValue(_uiState.value?.copy(isStreamingCam50 = false))
            } else if (cameraId == "51") {
                _uiState.postValue(_uiState.value?.copy(isStreamingCam51 = false))
            }
            postEvent("Failed to start streaming for Cam $cameraId: ${e.message}")
        }
    }

    fun stopStreaming(cameraId: String) {
        val streamState = cameraStreams[cameraId] ?: run {
            Log.e(TAG, "No stream state for $cameraId in stopStreaming.")
            return
        }
        if (!streamState.isStreaming) {
            Log.w(TAG, "Camera $cameraId is not currently streaming.")
            return
        }
        Log.i(TAG, "Stopping streaming for Cam $cameraId...")

        try {
            streamState.streamer.disconnect()
        } catch (e: Exception) {
            Log.e(TAG, "Exception disconnecting streamer for $cameraId", e)
        }
        streamState.isStreaming = false // Set before cleaning up encoder

        // Update UI State
        if (cameraId == "50") {
            _uiState.postValue(_uiState.value?.copy(isStreamingCam50 = false))
        } else if (cameraId == "51") {
            _uiState.postValue(_uiState.value?.copy(isStreamingCam51 = false))
        }

        // Clean up encoder resources
        try {
            streamState.encoder?.stop()
            streamState.encoder?.release()
        } catch (e: Exception) {
            Log.e(TAG, "Exception stopping/releasing encoder for $cameraId", e)
        }
        streamState.encoder = null
        streamState.encoderInputSurface?.release() // Release the surface
        streamState.encoderInputSurface = null

        // Recreate session without encoder surface IF camera device is still valid
        if (streamState.cameraDevice != null) {
            createSessionInternal(cameraId, isStreaming = false)
        } else {
            Log.w(TAG, "CameraDevice for $cameraId is null, cannot reconfigure session in stopStreaming. It might have been closed or disconnected.")
        }
        postEvent("Streaming stopped for Cam $cameraId")
        Log.i(TAG, "Streaming stopped for Cam $cameraId.")
    }

    private fun cleanupStream(cameraId: String) {
        Log.d(TAG, "Cleaning up stream for camera $cameraId")
        val streamState = cameraStreams.remove(cameraId) // Remove from map and get the state
        streamState?.apply {
            isStreaming = false // Ensure isStreaming is false
            try { streamer.disconnect() } catch (e: Exception) { Log.e(TAG, "Error disconnecting streamer for $cameraId during cleanup", e)}

            try { encoder?.stop() } catch (e: Exception) { Log.e(TAG, "Error stopping encoder for $cameraId during cleanup", e)}
            try { encoder?.release() } catch (e: Exception) { Log.e(TAG, "Error releasing encoder for $cameraId during cleanup", e)}
            encoder = null // Ensure it's nullified
            try { encoderInputSurface?.release() } catch (e: Exception) { Log.e(TAG, "Error releasing encoder surface for $cameraId during cleanup", e)}
            encoderInputSurface = null // Ensure it's nullified

            try { cameraSession?.close() } catch (e: Exception) { Log.e(TAG, "Error closing session for $cameraId during cleanup", e)}
            cameraSession = null // Ensure it's nullified

            try { cameraDevice?.close() } catch (e: Exception) { Log.e(TAG, "Error closing device for $cameraId during cleanup", e)}
            cameraDevice = null // Ensure it's nullified
            // previewSurface is owned by TextureView, no need to release here from ViewModel's perspective.
        }

        // Update UI state for the cleaned-up camera
        if (cameraId == "50") {
            _uiState.postValue(_uiState.value?.copy(isStreamingCam50 = false, showParametersCam50 = false, intrinsicsTextCam50 = "NA", extrinsicsTextCam50 = "NA"))
        } else if (cameraId == "51") {
            _uiState.postValue(_uiState.value?.copy(isStreamingCam51 = false, showParametersCam51 = false, intrinsicsTextCam51 = "NA", extrinsicsTextCam51 = "NA"))
        }
        Log.i(TAG, "Stream cleanup for $cameraId finished.")
    }


    fun onPause() {
        Log.d(TAG, "onPause called in ViewModel.")
        stopAllStreams()
    }

    fun stopAllStreams() {
        Log.i(TAG, "Stopping all camera streams.")
        // Iterate over a copy of keys to avoid ConcurrentModificationException as cleanupStream modifies the map
        val streamIds = ArrayList(cameraStreams.keys)
        streamIds.forEach { cameraId ->
            Log.d(TAG, "Cleaning up stream for camera ID: $cameraId from stopAllStreams")
            cleanupStream(cameraId) // This will also remove it from cameraStreams map
        }
        // cameraStreams.clear() // Not needed if cleanupStream removes items
        Log.i(TAG, "All camera streams cleaned up. cameraStreams map should be empty: ${cameraStreams.isEmpty()}")
    }


    fun toggleParametersDisplay(cameraId: String) {
        val currentUiState = _uiState.value ?: CameraUiState()
        // Try to get activeConfig from an open stream first, then from the general availableConfigs list
        val streamState = cameraStreams[cameraId]
        val configToUse = streamState?.activeConfig ?: currentUiState.availableCameras.find{ it.id == cameraId }

        if (cameraId == "50") {
            val newShowParamsState = !(currentUiState.showParametersCam50)
            var newIntrinsicsText = currentUiState.intrinsicsTextCam50
            var newExtrinsicsText = currentUiState.extrinsicsTextCam50
            if (newShowParamsState && configToUse != null) {
                newIntrinsicsText = configToUse.intrinsics?.let {
                    """
                    Fx: ${it.focalLengthX}
                    Fy: ${it.focalLengthY}
                    Cx: ${it.principalPointX}
                    Cy: ${it.principalPointY}
                    Dist: ${it.distortionCoefficients?.contentToString() ?: "N/A"}
                    """.trimIndent()
                } ?: "Intrinsics N/A"
                newExtrinsicsText = configToUse.extrinsics?.let {
                    """
                    Rot: ${it.rotationQuaternion.contentToString()}
                    Trans: ${it.translationVector.contentToString()}
                    """.trimIndent()
                } ?: "Extrinsics N/A"
            } else if (!newShowParamsState) { // Reset to N/A if hiding
                newIntrinsicsText = "N/A"
                newExtrinsicsText = "N/A"
            }
            _uiState.postValue(currentUiState.copy(
                showParametersCam50 = newShowParamsState,
                intrinsicsTextCam50 = newIntrinsicsText,
                extrinsicsTextCam50 = newExtrinsicsText
            ))
        } else if (cameraId == "51") {
            val newShowParamsState = !(currentUiState.showParametersCam51)
            var newIntrinsicsText = currentUiState.intrinsicsTextCam51
            var newExtrinsicsText = currentUiState.extrinsicsTextCam51
            if (newShowParamsState && configToUse != null) {
                newIntrinsicsText = configToUse.intrinsics?.let {
                    """
                    Fx: ${it.focalLengthX}
                    Fy: ${it.focalLengthY}
                    Cx: ${it.principalPointX}
                    Cy: ${it.principalPointY}
                    Dist: ${it.distortionCoefficients?.contentToString() ?: "N/A"}
                    """.trimIndent()
                } ?: "Intrinsics N/A"
                newExtrinsicsText = configToUse.extrinsics?.let {
                    """
                    Rot: ${it.rotationQuaternion.contentToString()}
                    Trans: ${it.translationVector.contentToString()}
                    """.trimIndent()
                } ?: "Extrinsics N/A"
            } else if (!newShowParamsState) { // Reset to N/A if hiding
                newIntrinsicsText = "N/A"
                newExtrinsicsText = "N/A"
            }
            _uiState.postValue(currentUiState.copy(
                showParametersCam51 = newShowParamsState,
                intrinsicsTextCam51 = newIntrinsicsText,
                extrinsicsTextCam51 = newExtrinsicsText
            ))
        }
    }


    fun shutdown() {
        Log.d(TAG, "shutdown() called in ViewModel.")
        stopAllStreams()
        // Threads will be quit in onCleared
    }


    override fun onCleared() {
        super.onCleared()
        Log.d(TAG, "ViewModel onCleared. Ensuring all streams are stopped and quitting threads.")
        shutdown() // Ensure streams are stopped before quitting threads
        cameraThread.quitSafely()
        encoderThread.quitSafely()
        Log.i(TAG, "Threads quit safely.")
    }

    private val executor: Executor = Executor { cameraHandler.post(it) }
}