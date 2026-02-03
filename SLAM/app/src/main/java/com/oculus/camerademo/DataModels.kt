@file:OptIn(InternalSerializationApi::class)
package com.oculus.camerademo
import kotlinx.serialization.InternalSerializationApi
import kotlinx.serialization.Serializable
data class CameraUiState(
    val availableCameras: List<CameraConfig> = emptyList(),

    // Camera 50 specific UI data
    val showParametersCam50: Boolean = false,
    val intrinsicsTextCam50: String = "N/A",
    val extrinsicsTextCam50: String = "N/A",
    val isStreamingCam50: Boolean = false,

    // Camera 51 specific UI data
    val showParametersCam51: Boolean = false,
    val intrinsicsTextCam51: String = "N/A",
    val extrinsicsTextCam51: String = "N/A",
    val isStreamingCam51: Boolean = false,

    //example field, i may need to change later but works for now
    val cameraBrightness: Int = 0,
    val activeCameraWidth: Int = 0,
    val activeCameraHeight: Int = 0
)

data class CameraConfig(
    val id: String,
    val width: Int,
    val height: Int,
    val position: String,
    val isPassthrough: Boolean,
    val intrinsics: CameraIntrinsics?,
    val extrinsics: CameraExtrinsics?
)

@Serializable
data class CameraIntrinsics(
    val focalLengthX: Float,
    val focalLengthY: Float,
    val principalPointX: Float,
    val principalPointY: Float,
    val distortionCoefficients: FloatArray?
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CameraIntrinsics

        if (focalLengthX != other.focalLengthX) return false
        if (focalLengthY != other.focalLengthY) return false
        if (principalPointX != other.principalPointX) return false
        if (principalPointY != other.principalPointY) return false
        if (distortionCoefficients != null) {
            if (other.distortionCoefficients == null) return false
            if (!distortionCoefficients.contentEquals(other.distortionCoefficients)) return false
        } else if (other.distortionCoefficients != null) return false

        return true
    }

    override fun hashCode(): Int {
        var result = focalLengthX.hashCode()
        result = 31 * result + focalLengthY.hashCode()
        result = 31 * result + principalPointX.hashCode()
        result = 31 * result + principalPointY.hashCode()
        result = 31 * result + (distortionCoefficients?.contentHashCode() ?: 0)
        return result
    }
}

@Serializable
data class CameraExtrinsics(
    val rotationQuaternion: FloatArray,
    val translationVector: FloatArray
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CameraExtrinsics

        if (!rotationQuaternion.contentEquals(other.rotationQuaternion)) return false
        if (!translationVector.contentEquals(other.translationVector)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = rotationQuaternion.contentHashCode()
        result = 31 * result + translationVector.contentHashCode()
        return result
    }
}

@Serializable
data class FrameMetadata(
    val timestamp: Long, // nanoseconds for ROS
    val intrinsics: CameraIntrinsics?,
    val extrinsics: CameraExtrinsics?
)

// CameraEvent remains the same, but the ViewModel will use NotificationEvent directly
sealed class CameraEvent {
    data class NotificationEvent(val message: String) : CameraEvent()
}