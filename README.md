# Meta Quest Dual Camera Streaming Demo

**Author:** Layla Ashry
**Platform:** Meta Quest (Android / Horizon OS)

---

## Overview

This project implements a **dual-camera capture and streaming pipeline** on Meta Quest devices using the Meta Passthrough Camera API and Android’s Camera2 / MediaCodec stack. The application captures video from two passthrough cameras (camera IDs **50** and **51**), displays live previews using Jetpack Compose, and streams encoded video frames together with structured metadata to an external host over TCP.

---
## High-Level Architecture

The system is composed of four primary layers:

1. **UI & Lifecycle Layer (MainActivity / Compose UI)**

   * Handles Android lifecycle events and permission management
   * Hosts Jetpack Compose UI for dual camera previews and controls
   * Owns `TextureView` surfaces used by the camera pipeline

2. **Camera & State Management Layer (ViewModel)**

   * Discovers available passthrough cameras
   * Manages camera configuration, brightness, intrinsics, and extrinsics
   * Coordinates camera start/stop behavior based on UI and lifecycle state

3. **Encoding & Frame Preparation Layer**

   * Uses `MediaCodec` to encode camera frames
   * Extracts encoded buffers and per-frame metadata
   * Prepares frames for network transmission

4. **Networking & Streaming Layer (Streamer)**

   * Maintains a TCP socket connection to a remote host
   * Runs a dedicated background thread for network I/O
   * Packages encoded video frames with JSON-serialized metadata
   * Ensures backpressure via a bounded frame queue

---

## Key Components

### MainActivity.kt

* Entry point of the application
* Manages runtime permissions (Android camera + Meta/HZOS permissions)
* Coordinates surface readiness and camera startup
* Bridges Compose UI callbacks with ViewModel logic
* Ensures proper cleanup during lifecycle transitions

### CameraDemoViewModel (and related classes)

* Central state holder for camera configuration and streaming status
* Exposes observable UI state for Compose
* Starts and stops individual or dual camera streams
* Owns camera discovery results and parameter data

### Streamer.kt

* Low-level networking utility responsible for streaming data off-device
* Uses TCP sockets for deterministic, ordered delivery
* Sends each frame as:

  1. Metadata size (Int)
  2. Video frame size (Int)
  3. JSON metadata payload
  4. Encoded video bytes

This design allows external consumers (e.g., Python, C++, ROS nodes) to reconstruct frames and metadata reliably.

---

## Metadata Handling

Each video frame is accompanied by structured metadata (e.g., timestamps, camera parameters). Metadata is serialized using `kotlinx.serialization` to JSON, enabling:

* Language-agnostic decoding
* Easy logging and debugging
* Direct integration with robotics and vision pipelines

---

## Permissions and Platform Notes

* Requires standard Android `CAMERA` permission
* Requires Meta/Horizon OS passthrough camera permissions
* Designed specifically for Meta Quest devices supporting the Passthrough Camera API

---

## Tools, SDKs, and References

This project was developed with the support of the following tools and documentation:

* **Meta Passthrough Camera API Samples**
  [https://github.com/meta-quest/Meta-Passthrough-Camera-API-Samples](https://github.com/meta-quest/Meta-Passthrough-Camera-API-Samples)

* **Meta Spatial SDK – Passthrough Camera API Overview**
  [https://developers.meta.com/horizon/documentation/spatial-sdk/spatial-sdk-pca-overview](https://developers.meta.com/horizon/documentation/spatial-sdk/spatial-sdk-pca-overview)

* **Gemini** — used for ideation, debugging assistance, documenentation, and architectural reasoning

* **NVIDIA Documentation**
  Referenced for MediaCodec usage patterns, video encoding concepts, and performance considerations

---

# Tcp_dserver_node ROS

````
import rclpy  
from rclpy.node import Node  
import socket  
import struct  
import json  
from sensor_msgs.msg import CompressedImage, CameraInfo  
from geometry_msgs.msg import TransformStamped   
from builtin_interfaces.msg import Time   
import tf2_ros   
import traceback   
import threading    

# QoS-related imports used to configure ROS2 publishers for streaming data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy   


class QuestStreamReceiver(Node):
    """
    ROS 2 node that acts as a TCP server to receive streamed camera data
    from a Meta Quest device and republish it into ROS topics.

    Responsibilities:
    - Accept a TCP connection from the Quest headset
    - Receive binary-framed data containing:
        * timestamp (ms)
        * camera intrinsics (JSON)
        * camera extrinsics (JSON)
        * encoded image frame (bytes)
    - Publish:
        * sensor_msgs/CompressedImage
        * sensor_msgs/CameraInfo
        * TF transform for the camera pose
    """

    def __init__(self):
        # Initialize the ROS2 node with a fixed name
        super().__init__('quest_stream_receiver')

        # Declare configurable parameters for TCP server binding
        self.declare_parameter('listen_ip', '0.0.0.0')
        self.declare_parameter('listen_port', 8888)

        # Retrieve parameter values
        self.ip = self.get_parameter('listen_ip').get_parameter_value().string_value
        self.port = self.get_parameter('listen_port').get_parameter_value().integer_value

        # QoS profile optimized for low-latency video streaming:
        # - BEST_EFFORT: drop frames rather than blocking
        # - depth=1: keep only the most recent frame
        video_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher for compressed image data received over TCP
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/left/image_raw/compressed/from_tcp',
            video_qos_profile)

        # Publisher for camera intrinsic parameters
        self.cam_info_pub = self.create_publisher(
            CameraInfo,
            '/left/camera_info',
            video_qos_profile)

        # TF broadcaster for publishing the camera pose
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Log server startup information
        self.get_logger().info(
            f"TCP Server Node initialized. Listening on {self.ip}:{self.port}"
        )


    def run_server(self):
        """
        Main TCP server loop.
        - Binds to the configured IP and port
        - Accepts a single client connection at a time
        - Blocks until the client disconnects
        """

        # Create a TCP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Allow immediate rebinding after restart
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind and start listening
            s.bind((self.ip, self.port))
            s.listen()

            # Accept connections while ROS is running
            while rclpy.ok():
                self.get_logger().info(
                    "Waiting for a connection from the Quest headset..."
                )
                try:
                    conn, addr = s.accept()
                    # Handle a single connection until it closes
                    with conn:
                        self.get_logger().info(
                            f"Connection established with {addr}"
                        )
                        self.handle_connection(conn)
                except Exception as e:
                    # Catch unexpected server-level errors
                    self.get_logger().error(
                        f"Server loop error: {e}\n{traceback.format_exc()}"
                    )

                # Connection ended; wait for a new client
                self.get_logger().warn(
                    "Client disconnected. Waiting for new connection."
                )


    def handle_connection(self, conn):
        """
        Handles a connected TCP client.
        Continuously reads frame packets until the connection is lost.

        Expected packet format per frame:
        - 20-byte header:
            uint64 timestamp_ms
            uint32 intrinsics_size
            uint32 extrinsics_size
            uint32 frame_size
        - intrinsics JSON (intrinsics_size bytes)
        - extrinsics JSON (extrinsics_size bytes)
        - encoded image frame (frame_size bytes)
        """

        while rclpy.ok():
            try:
                # Read fixed-size header
                header_data = self._read_exact(conn, 20)
                if not header_data:
                    break

                # Unpack header fields (big-endian)
                timestamp_ms, intrinsics_size, extrinsics_size, frame_size = struct.unpack(
                    '>QIII', header_data
                )

                # Read variable-sized payloads
                intrinsics_bytes = self._read_exact(conn, intrinsics_size)
                extrinsics_bytes = self._read_exact(conn, extrinsics_size)
                frame_bytes = self._read_exact(conn, frame_size)

                # Exit if any payload is missing
                if not (intrinsics_bytes and extrinsics_bytes and frame_bytes):
                    break

                # Convert millisecond timestamp into ROS Time
                ros_time = Time(
                    sec=int(timestamp_ms // 1000),
                    nanosec=int((timestamp_ms % 1000) * 1_000_000)
                )

                # Decode JSON metadata
                intrinsics_data = json.loads(intrinsics_bytes)
                extrinsics_data = json.loads(extrinsics_bytes)

                # Publish received data into ROS
                self.publish_image(ros_time, frame_bytes)
                self.publish_camera_info(ros_time, intrinsics_data)
                self.publish_transform(ros_time, extrinsics_data)

            except (ConnectionResetError, BrokenPipeError) as e:
                # Handle normal connection termination
                self.get_logger().warn(f"Connection lost: {e}")
                break
            except Exception as e:
                # Handle malformed packets or unexpected errors
                self.get_logger().error(
                    f"Error handling connection: {e}\n{traceback.format_exc()}"
                )
                break


    def _read_exact(self, conn, length):
        """
        Utility function to read exactly `length` bytes from a socket.
        Ensures full payload reception despite TCP fragmentation.
        """
        data = bytearray()
        while len(data) < length:
            packet = conn.recv(length - len(data))
            if not packet:
                return None
            data.extend(packet)
        return bytes(data)


    def publish_image(self, timestamp, frame_data):
        """
        Publishes a compressed image message using the received frame bytes.
        """
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'quest_left_camera'
        msg.format = "h24"  # Intended to describe the encoded video format
        msg.data = list(frame_data)
        self.image_pub.publish(msg)


    def publish_camera_info(self, timestamp, intrinsics):
        """
        Publishes camera intrinsic parameters extracted from JSON metadata.
        """
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'quest_left_camera'

        # Image resolution (currently hardcoded)
        msg.width, msg.height = 1280, 720

        # Camera intrinsic matrix (K)
        msg.k = [
            float(intrinsics.get('focalLengthX', 0.0)), 0.0,
            float(intrinsics.get('principalPointX', 0.0)),
            0.0, float(intrinsics.get('focalLengthY', 0.0)),
            float(intrinsics.get('principalPointY', 0.0)),
            0.0, 0.0, 1.0
        ]

        # Distortion parameters (if present)
        distortion = intrinsics.get('distortionCoefficients', [])
        if distortion:
            msg.d = [float(c) for c in distortion]
            msg.distortion_model = "plumb_bob"
        else:
            msg.distortion_model = ""

        # Rectification matrix (identity)
        msg.r = [1.0] * 9

        # Projection matrix (P)
        msg.p = [
            msg.k[0], 0.0, msg.k[2], 0.0,
            0.0, msg.k[4], msg.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.cam_info_pub.publish(msg)


    def publish_transform(self, timestamp, extrinsics):
        """
        Publishes a TF transform describing the camera pose in the world frame.
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'quest_left_camera'

        # Extract translation and rotation from extrinsics JSON
        translation = extrinsics.get('translationVector', [0.0] * 3)
        rotation = extrinsics.get('rotationQuaternion', [0.0] * 3 + [1.0])

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    Entry point for the ROS2 node.
    Starts the TCP server in a background thread while spinning ROS callbacks.
    """
    rclpy.init(args=args)
    node = QuestStreamReceiver()

    # Run the blocking TCP server in a daemon thread
    server_thread = threading.Thread(target=node.run_server)
    server_thread.daemon = True
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
