# 🎧 audio_listener (ROS 2)

`audio_listener` is a ROS 2 Python package that captures audio from a microphone using PyAudio, buffers it, and publishes 1-second audio chunks as ROS messages. This package is ideal for applications involving voice commands, speech recognition, audio analysis, and more.

---

## 📦 Package Overview

This package reads raw PCM audio data from an input device (e.g. a microphone), converts it into `Int16MultiArray` messages, and publishes them to the `~/audio` topic.

---

## 🧠 Features

- Captures microphone input using `pyaudio`.
- Buffers and publishes audio in 1-second intervals.
- Publishes to `std_msgs/Int16MultiArray` on the topic `~/audio`.
- Configurable parameters via ROS 2 parameters.

---

## 📁 File Breakdown

### `audio_listener/audio_listener.py`
The main ROS node:
- Initializes ROS 2 parameters (`channels`, `rate`, `frames_per_buffer`, `device_index`).
- Opens a PyAudio stream to capture audio.
- Buffers audio data in real-time.
- Every 1 second of audio is published as an `Int16MultiArray`.

### `launch/audio_listener.launch.py`
Launch file to start the node:
```bash
ros2 launch audio_listener audio_listener.launch.py
```

### `package.xml`
ROS 2 package manifest with metadata and test dependencies. Declares ament_python as the build type.

### `setup.py`
Python setup file for installing the package and registering the node:

```bash
ros2 run audio_listener audio_listener
```

---

## 🔁 Workflow

### 1. Build the package
```bash
colcon build --packages-select audio_listener
source install/setup.bash
```

### 2. Launch the node
```bash
ros2 launch audio_listener audio_listener.launch.py
```

### 3. What Happens
```
Microphone → PyAudio → NumPy buffer → ROS2 message → ~/audio topic
```
Audio is read in chunks (e.g. 1000 frames per buffer).

Once 1 second of audio (e.g. 16000 samples at 16kHz) is collected:

It’s published to the ~/audio topic.

---

## 📡 Topic Info

- **Topic:** `~/audio`
- **Type:** `std_msgs/msg/Int16MultiArray`
- **Content:** 1 second of audio samples (`rate` samples of type `int16`)

---

## ⚙️ Parameters

| Name              | Type | Default | Description                             |
|-------------------|------|---------|-----------------------------------------|
| `channels`         | int  | 1       | Number of audio input channels          |
| `frames_per_buffer`| int  | 1000    | Frames read per buffer                  |
| `rate`             | int  | 16000   | Sample rate in Hz                       |
| `device_index`     | int  | -1      | Index of the audio input device (-1 = default) |

---

## 🧪 Example Use Case

This node can be used as the first step in a pipeline such as:

```
audio_listener → speech_recognizer → text → NLP → robot control
```

---

## 🧼 Cleanup

On shutdown, the node:

- Closes the PyAudio stream.
- Terminates the PyAudio instance cleanly.

---

## 🔧 Future Extensions

- Add real-time speech-to-text recognition.
- Record and save audio to disk.
- Integrate with sound classification models.
- Visualize audio waveforms in real time.