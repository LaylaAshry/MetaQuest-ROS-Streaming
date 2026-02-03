// MainActivity.kt
package com.oculus.camerademo

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.os.Bundle
import android.util.Log
import android.view.Surface
import android.view.TextureView
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.Button
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.ViewModelProvider
import com.oculus.camerademo.ui.theme.CameraDemoTheme
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
// MainActivity.kt
// By: Layla Ashry
// Entry point and primary UI controller for the Camera Demo application running on Meta Quest.
// This activity:
//
// - Initializes and coordinates with CameraDemoViewModel
// - Hosts the Jetpack Compose UI for dual camera previews (camera IDs "50" and "51")
// - Owns TextureView surfaces used for Camera2 preview and MediaCodec streaming
// - Starts and stops dual camera streams once permissions and surfaces are ready
// - Handles lifecycle events to ensure proper camera and resource cleanup
//
// The activity bridges Android lifecycle management, Compose UI interaction,
// and low-level camera streaming logic implemented in the ViewModel.

class MainActivity : ComponentActivity() {
    private val viewModel by lazy {
        ViewModelProvider.AndroidViewModelFactory(application)
            .create(CameraDemoViewModel::class.java)
    }
    private val permissions = arrayOf(
        Manifest.permission.CAMERA,
        PermissionManager.HZOS_CAMERA_PERMISSION
    )

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissionsResult ->
        if (permissionsResult.all { it.value }) {
            Log.d(
                "MainActivity",
                "Permissions granted by user. Attempting to start dual cameras if surfaces are ready."
            )
            tryStartDualCameras()
        } else {
            Toast.makeText(this, "Camera permissions required", Toast.LENGTH_SHORT).show()
        }
    }
    private var surfaceCam50: Surface? = null
    private var surfaceTextureCam50: SurfaceTexture? = null
    private var surfaceCam51: Surface? = null
    private var surfaceTextureCam51: SurfaceTexture? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Log.d("MainActivityLifecycle", "onCreate") // lifecycle Log
        viewModel.init()
        viewModel.uiState.observe(this) { uiState ->
            setContent {
                CameraApp(
                    uiState = uiState,
                    onCheckPermissionsAndStart = { checkPermissions() },
                    onStopAllCameras = { viewModel.stopAllStreams() },
                    onExit = { finish() },
                    onToggleParamsCam50 = { viewModel.toggleParametersDisplay("50") },
                    onToggleStreamingCam50 = { toggleStreamingForCamera("50") },
                    onSurfaceAvailableCam50 = { surfaceTexture, _, _ ->
                        updatePreviewTextureCam50(surfaceTexture)
                    },
                    onToggleParamsCam51 = { viewModel.toggleParametersDisplay("51") },
                    onToggleStreamingCam51 = { toggleStreamingForCamera("51") },
                    onSurfaceAvailableCam51 = { surfaceTexture, _, _ ->
                        updatePreviewTextureCam51(surfaceTexture)
                    }
                )
            }
        }

        viewModel.cameraEvents.observe(this) { event ->
            if (event is CameraEvent.NotificationEvent) {
                Toast.makeText(this, event.message, Toast.LENGTH_SHORT).show()
            }
        }
    }

    override fun onStart() {
        super.onStart()
        Log.d("MainActivityLifecycle", "onStart")
    }

    override fun onResume() {
        super.onResume()
        Log.d("MainActivityLifecycle", "onResume")
    }


    override fun onPause() {
        Log.d("MainActivityLifecycle", "onPause")
        super.onPause()
        viewModel.onPause()
    }


    override fun onStop() {
        Log.d("MainActivityLifecycle", "onStop")
        super.onStop()
        viewModel.shutdown()
    }

    override fun onDestroy() {
        Log.d("MainActivityLifecycle", "onDestroy")
        super.onDestroy()
        surfaceCam50?.release()
        surfaceCam51?.release()
    }

    private fun toggleStreamingForCamera(cameraId: String) {
        val computerIpAddress = "192.168.6.29"
        val port = if (cameraId == "50") 8888 else 8889

        val isCurrentlyStreaming = if (cameraId == "50") {
            viewModel.uiState.value?.isStreamingCam50 == true
        } else {
            viewModel.uiState.value?.isStreamingCam51 == true
        }

        if (isCurrentlyStreaming) {
            viewModel.stopStreaming(cameraId)
        } else {
            val surfaceToCheck = if (cameraId == "50") surfaceCam50 else surfaceCam51
            if (surfaceToCheck != null) {
                viewModel.startStreaming(cameraId, computerIpAddress, port)
            } else {
                Toast.makeText(
                    this,
                    "Camera $cameraId preview not started yet.",
                    Toast.LENGTH_SHORT
                ).show()
            }
        }
    }

    private fun checkPermissions() {
        when {
            hasAllPermissions() -> {
                Log.d(
                    "MainActivity",
                    "Permissions already granted. Attempting to start dual cameras if surfaces are ready."
                )
                tryStartDualCameras()
            }

            else -> requestPermissionLauncher.launch(permissions)
        }
    }

    private fun hasAllPermissions(): Boolean {
        return permissions.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun updatePreviewTextureCam50(surfaceTexture: SurfaceTexture) {
        surfaceCam50?.release()
        surfaceTextureCam50 = surfaceTexture
        surfaceCam50 = Surface(surfaceTexture)
        Log.d("MainActivity", "Surface for Cam 50 updated.")
        tryStartDualCameras()
    }

    private fun updatePreviewTextureCam51(surfaceTexture: SurfaceTexture) {
        surfaceCam51?.release()
        surfaceTextureCam51 = surfaceTexture
        surfaceCam51 = Surface(surfaceTexture)
        Log.d("MainActivity", "Surface for Cam 51 updated.")
        tryStartDualCameras()
    }

    private fun tryStartDualCameras() {
        val s50 = surfaceCam50
        val s51 = surfaceCam51

        if (s50 != null && s51 != null) {
            if (hasAllPermissions()) {

                val availableCameras = viewModel.uiState.value?.availableCameras
                if (availableCameras == null || availableCameras.none { it.id == "50" } || availableCameras.none { it.id == "51" }) {
                    Log.w(
                        "MainActivity",
                        "tryStartDualCameras: ViewModel camera configs not yet populated or missing 50/51. Will retry or wait for UI state update."
                    )
                    return
                }


                Log.i(
                    "MainActivity",
                    "Both surfaces ready, permissions granted, and ViewModel has camera list. Starting dual camera streams."
                )
                viewModel.startDualCameraStreams(s50, s51)
            } else {
                Log.w(
                    "MainActivity",
                    "Both surfaces ready, but permissions not granted. Requesting permissions." //should help debug permissions stuff
                )
                checkPermissions()
            }
        } else {
            Log.d(
                "MainActivity",
                "tryStartDualCameras: Waiting for both surfaces to be ready (Cam50: ${s50 != null}, Cam51: ${s51 != null})"
            )
        }
    }


    @Composable
    fun CameraApp(
        uiState: CameraUiState,
        onCheckPermissionsAndStart: () -> Unit,
        onStopAllCameras: () -> Unit,
        onExit: () -> Unit,
        onToggleParamsCam50: () -> Unit,
        onToggleStreamingCam50: () -> Unit,
        onSurfaceAvailableCam50: (SurfaceTexture, Int, Int) -> Unit,
        onToggleParamsCam51: () -> Unit,
        onToggleStreamingCam51: () -> Unit,
        onSurfaceAvailableCam51: (SurfaceTexture, Int, Int) -> Unit
    ) {
        CameraDemoTheme {
            Scaffold(modifier = Modifier.fillMaxSize()) { padding ->
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(padding)
                        .imePadding()
                        .verticalScroll(rememberScrollState()),
                    verticalArrangement = Arrangement.Top
                ) {

                    Text("Camera 50", modifier = Modifier.align(Alignment.CenterHorizontally))
                    CameraPreview(
                        onSurfaceAvailable = onSurfaceAvailableCam50
                    )
                    CameraControls(
                        isStreaming = uiState.isStreamingCam50,
                        showParameters = uiState.showParametersCam50,
                        cameraBrightness = uiState.cameraBrightness,
                        intrinsicsText = uiState.intrinsicsTextCam50,
                        extrinsicsText = uiState.extrinsicsTextCam50,
                        onStart = onCheckPermissionsAndStart,
                        onStop = onStopAllCameras,
                        onToggleParams = onToggleParamsCam50,
                        onToggleStreaming = onToggleStreamingCam50
                    )

                    Spacer(modifier = Modifier.height(16.dp))

                    Text("Camera 51", modifier = Modifier.align(Alignment.CenterHorizontally))
                    CameraPreview(
                        onSurfaceAvailable = onSurfaceAvailableCam51
                    )
                    CameraControls(
                        isStreaming = uiState.isStreamingCam51,
                        showParameters = uiState.showParametersCam51,
                        cameraBrightness = uiState.cameraBrightness,
                        intrinsicsText = uiState.intrinsicsTextCam51,
                        extrinsicsText = uiState.extrinsicsTextCam51,
                        onStart = onCheckPermissionsAndStart,
                        onStop = onStopAllCameras,
                        onToggleParams = onToggleParamsCam51,
                        onToggleStreaming = onToggleStreamingCam51
                    )

                    Button(
                        onClick = onExit,
                        modifier = Modifier
                            .align(Alignment.CenterHorizontally)
                            .padding(16.dp)
                    ) {
                        Text("Exit")
                    }
                }
            }
        }
    }


    @Composable
    fun CameraControls(
        isStreaming: Boolean,
        showParameters: Boolean,
        cameraBrightness: Int,
        intrinsicsText: String,
        extrinsicsText: String,
        onStart: () -> Unit,
        onStop: () -> Unit,
        onToggleParams: () -> Unit,
        onToggleStreaming: () -> Unit
    ) {
        Column(modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp)) {
            Row(
                horizontalArrangement = Arrangement.SpaceEvenly,
                modifier = Modifier.fillMaxWidth()
            ) {
                Button(onClick = onStart) { Text("Permissions") }
                Button(onClick = onStop) { Text("Stop All") }
                Button(onClick = onToggleStreaming) {
                    Text(if (isStreaming) "Stop Stream" else "Start Stream")
                }
            }
            Spacer(modifier = Modifier.height(10.dp))
            Row(
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.fillMaxWidth()
            ) {
                Column {
                    Text("Brightness:", fontWeight = FontWeight.Bold)
                    Text(cameraBrightness.toString())
                }
                Button(onClick = onToggleParams) {
                    Text(if (showParameters) "Hide Params" else "Show Params")
                }
            }
            if (showParameters) {
                Column(
                    modifier = Modifier
                        .padding(vertical = 8.dp)
                        .background(Color.LightGray.copy(alpha = 0.2f))
                        .fillMaxWidth()
                ) {
                    Text(
                        "Intrinsics:",
                        fontWeight = FontWeight.Bold,
                        modifier = Modifier.padding(horizontal = 8.dp)
                    )
                    Text(intrinsicsText, modifier = Modifier.padding(horizontal = 8.dp))
                    Spacer(modifier = Modifier.height(8.dp))
                    Text(
                        "Extrinsics:",
                        fontWeight = FontWeight.Bold,
                        modifier = Modifier.padding(horizontal = 8.dp)
                    )
                    Text(extrinsicsText, modifier = Modifier.padding(horizontal = 8.dp))
                }
            }
        }
    }


    @Composable
    fun CameraPreview(onSurfaceAvailable: (SurfaceTexture, Int, Int) -> Unit) {
        AndroidView(
            factory = { context ->
                TextureView(context).apply {
                    surfaceTextureListener = object : TextureView.SurfaceTextureListener {
                        override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) {
                            onSurfaceAvailable(st, w, h)
                        }

                        override fun onSurfaceTextureSizeChanged(
                            st: SurfaceTexture,
                            w: Int,
                            h: Int
                        ) {  onSurfaceAvailable(
                            st,
                            w,
                            h
                        )
                        } // potentially re-trigger with new size

                        override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
                            //Signal surface is gone
                            Log.d("CameraPreview", "SurfaceTextureDestroyed for $this")
                            return true
                        }

                        override fun onSurfaceTextureUpdated(surface: SurfaceTexture) {}
                    }
                }
            },
            modifier = Modifier
                .fillMaxWidth()
                .aspectRatio(16f / 9f) // MODIFIED from 4f / 3f
        )
    }
}