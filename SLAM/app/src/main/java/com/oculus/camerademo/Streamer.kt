// Streamer.kt
package com.oculus.camerademo

import android.media.MediaCodec
import android.util.Log
// --- CHANGE 1: ADD IMPORTS FOR JSON SERIALIZATION ---
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.DataOutputStream
import java.io.IOException
import java.net.Socket
import java.nio.ByteBuffer
import java.util.concurrent.ArrayBlockingQueue

// Streamer.kt
//
// Author: Layla Ashry
//
// Network streaming utility responsible for transmitting encoded video frames
// and associated camera metadata from the Meta Quest device to an external host.
//
// This class:
//
// - Manages a TCP socket connection to a remote computer
// - Runs a dedicated background thread for network I/O
// - Packages MediaCodec-encoded video frames with structured metadata
// - Serializes per-frame metadata to JSON using kotlinx.serialization
// - Maintains a bounded queue to decouple camera encoding from network transmission
// - Ensures thread-safe connection lifecycle management and graceful cleanup
//
// The Streamer acts as the final stage of the camera pipeline, bridging
// Android MediaCodec output with an external processing system (e.g., ROS,
// vision pipelines, or logging tools).

class Streamer {
    private var networkSocket: Socket? = null
    private var dataOutputStream: DataOutputStream? = null
    private var networkThread: Thread? = null
    @Volatile private var isConnected = false

   //this class is frame structure
    private data class FramePacket(
        val buffer: ByteBuffer, //vidxeo frames
        val info: MediaCodec.BufferInfo,
        val metadataJson: String //metadata
    )
    private val frameQueue = ArrayBlockingQueue<FramePacket>(30)

    companion object {
        private const val TAG = "Streamer"
    }

    fun connect(ipAddress: String, port: Int) {
        if (isConnected) return
        isConnected = true
        networkThread = Thread {
            try {
                Log.d(TAG, "Attempting to connect to $ipAddress:$port")
                networkSocket = Socket(ipAddress, port)
                dataOutputStream = DataOutputStream(networkSocket!!.getOutputStream())
                Log.d(TAG, "Socket connected. Starting consumer loop.")
                while (isConnected && networkSocket?.isClosed == false) {
                    try {
                        val packet = frameQueue.take()
                        Log.d(TAG, "Dequeued frame packet. Metadata size=${packet.metadataJson.length}, video size=${packet.info.size}")
                        val metadataBytes = packet.metadataJson.toByteArray(Charsets.UTF_8)
                        val videoDataBytes = ByteArray(packet.info.size)
                        packet.buffer.get(videoDataBytes)
                        dataOutputStream?.writeInt(metadataBytes.size)
                        dataOutputStream?.writeInt(videoDataBytes.size)
                        dataOutputStream?.write(metadataBytes)
                        dataOutputStream?.write(videoDataBytes)

                        dataOutputStream?.flush()

                    } catch (e: InterruptedException) {
                        Log.d(TAG, "Network thread interrupted, disconnecting.")
                        Thread.currentThread().interrupt()
                        break
                    } catch (e: IOException) {
                        if (isConnected) {
                            Log.e(TAG, "Error sending frame data", e)
                        }
                        break // connection is lost prob
                    }
                }
            } catch (e: IOException) {
                if (isConnected) {
                    Log.e(TAG, "Socket connection error", e)
                }
            } finally {
                Log.d(TAG, "Network thread finishing, performing cleanup.")
                disconnectInternal()
            }
        }
        networkThread?.start()
    }

//data enteriing streamer
    fun queueFrame(buffer: ByteBuffer, info: MediaCodec.BufferInfo, metadata: FrameMetadata) {
        if (!isConnected || networkSocket?.isClosed == true) return

        // copy buffe
        val freshBuffer = ByteBuffer.allocate(info.size)
        val originalPosition = buffer.position()
        buffer.position(info.offset)
        freshBuffer.put(buffer)
        freshBuffer.flip()
        buffer.position(originalPosition)

        // put metadatya in json string
        val metadataJson = Json.encodeToString(metadata)

        // create the packet with the video data and the JSON string
        val packet = FramePacket(
            freshBuffer,
            info.apply { offset = 0 },
            metadataJson
        )

        if (!frameQueue.offer(packet)) {
            Log.w(TAG, "Dropping frame packet, queue is full.")
        }
    }

    private fun disconnectInternal() {
        isConnected = false
        networkThread?.interrupt()
        try {
            dataOutputStream?.close()
        } catch (_: IOException) {}
        try {
            networkSocket?.close()
        } catch (_: IOException) {}
        try {
            networkThread?.join(500)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
            Log.w(TAG, "Interrupted while joining network thread.")
        }
        frameQueue.clear()
        networkSocket = null
        dataOutputStream = null
        networkThread = null
        Log.d(TAG, "Streamer disconnected internally.")
    }

    fun disconnect() {
        Log.d(TAG, "Disconnect called.")
        if (!isConnected && networkThread == null && networkSocket == null) {
            Log.d(TAG, "Already disconnected or never connected.")
            return
        }
        disconnectInternal()
    }
}