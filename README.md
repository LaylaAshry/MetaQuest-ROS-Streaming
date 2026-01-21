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


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy   


class QuestStreamReceiver(Node):     
    def __init__(self):   
        super().__init__('quest_stream_receiver')   
        self.declare_parameter('listen_ip', '0.0.0.0')   
        self.declare_parameter('listen_port', 8888)   
        self.ip = self.get_parameter('listen_ip').get_parameter_value().string_value     
        self.port = self.get_parameter('listen_port').get_parameter_value().integer_value   


        video_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
       
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/left/image_raw/compressed/from_tcp',
            video_qos_profile)
           
        self.cam_info_pub = self.create_publisher(
            CameraInfo,
            '/left/camera_info',
            video_qos_profile)
        # --- END OF QOS FIX ---


        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info(f"TCP Server Node initialized. Listening on {self.ip}:{self.port}")


    def run_server(self):
        # ... (The rest of the file is identical and correct)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.ip, self.port))
            s.listen()
            while rclpy.ok():
                self.get_logger().info("Waiting for a connection from the Quest headset...")
                try:
                    conn, addr = s.accept()
                    with conn:
                        self.get_logger().info(f"Connection established with {addr}")
                        self.handle_connection(conn)
                except Exception as e:
                    self.get_logger().error(f"Server loop error: {e}\n{traceback.format_exc()}")
                self.get_logger().warn("Client disconnected. Waiting for new connection.")


    def handle_connection(self, conn):
        while rclpy.ok():
            try:
                header_data = self._read_exact(conn, 20)
                if not header_data: break
                timestamp_ms, intrinsics_size, extrinsics_size, frame_size = struct.unpack('>QIII', header_data)
                intrinsics_bytes = self._read_exact(conn, intrinsics_size)
                extrinsics_bytes = self._read_exact(conn, extrinsics_size)
                frame_bytes = self._read_exact(conn, frame_size)
                if not (intrinsics_bytes and extrinsics_bytes and frame_bytes): break
                ros_time = Time(sec=int(timestamp_ms // 1000), nanosec=int((timestamp_ms % 1000) * 1_000_000))
                intrinsics_data = json.loads(intrinsics_bytes)
                extrinsics_data = json.loads(extrinsics_bytes)
                self.publish_image(ros_time, frame_bytes)
                self.publish_camera_info(ros_time, intrinsics_data)
                self.publish_transform(ros_time, extrinsics_data)
            except (ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f"Connection lost: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"Error handling connection: {e}\n{traceback.format_exc()}")
                break


    def _read_exact(self, conn, length):
        data = bytearray()
        while len(data) < length:
            packet = conn.recv(length - len(data))
            if not packet: return None
            data.extend(packet)
        return bytes(data)


    def publish_image(self, timestamp, frame_data):
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'quest_left_camera'
        msg.format = "h24"
        msg.data = list(frame_data)
        self.image_pub.publish(msg)


    def publish_camera_info(self, timestamp, intrinsics):
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'quest_left_camera'
        msg.width, msg.height = 1280, 720
        msg.k = [
            float(intrinsics.get('focalLengthX', 0.0)), 0.0, float(intrinsics.get('principalPointX', 0.0)),
            0.0, float(intrinsics.get('focalLengthY', 0.0)), float(intrinsics.get('principalPointY', 0.0)),
            0.0, 0.0, 1.0
        ]
        distortion = intrinsics.get('distortionCoefficients', [])
        if distortion:
            msg.d = [float(c) for c in distortion]
            msg.distortion_model = "plumb_bob"
        else:
            msg.distortion_model = ""
        msg.r = [1.0] * 9
        msg.p = [msg.k[0], 0.0, msg.k[2], 0.0, 0.0, msg.k[4], msg.k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.cam_info_pub.publish(msg)


    def publish_transform(self, timestamp, extrinsics):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'quest_left_camera'
        translation = extrinsics.get('translationVector', [0.0]*3)
        rotation = extrinsics.get('rotationQuaternion', [0.0]*3 + [1.0])
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(translation[0]), float(translation[1]), float(translation[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):  
    rclpy.init(args=args)   
    node = QuestStreamReceiver()  
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


````
