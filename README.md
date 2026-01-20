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


