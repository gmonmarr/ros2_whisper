
# 🗣️ ROS 2 Whisper

ROS 2 integration for [`whisper.cpp`](https://github.com/ggerganov/whisper.cpp) enabling real-time or event-based speech-to-text transcription using modular ROS 2 components.

---

## 📦 Repository Structure

This monorepo is organized into modular ROS 2 packages:

- `audio_listener`: Captures audio from mic and publishes it.
- `whisper_util`: C++ utilities for whisper.cpp wrapping, model management, and audio buffering.
- `whisper_server`: Loads whisper.cpp and performs inference on incoming audio.
- `whisper_idl`: Contains `.msg` and `.action` definitions like `WhisperTokens`, `AudioTranscript`, and `Inference`.
- `whisper_bringup`: Launch logic for composing the full pipeline.
- `whisper_demos`: Simple interactive clients using the action and topics.

---

## 🐳 Docker Support

A `Dockerfile` is provided for isolated, reproducible development and deployment.

### 🔧 Build the container

```bash
docker build -t ros2_whisper .
```

### ▶️ Run with audio access

```bash
docker run --rm -it --net=host \
  --device /dev/snd \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_whisper
```

> Make sure you have permissions for `/dev/snd` (audio group), and optionally allow X11 forwarding for GUI audio tools.

---

## 🔁 Submodules

This repo uses Git submodules for dependencies like whisper.cpp:

```bash
git clone --recurse-submodules https://github.com/ros-ai/ros2_whisper.git
```

Or if already cloned:

```bash
git submodule update --init --recursive
```

---

## 🧪 Live Demo: Harry Potter Transcription

This example shows live transcription of the 6th chapter in ***Harry Potter and the Philosopher's Stone*** from Audible:

![harry_potter_sample](./doc/harry_potter_sample.gif)

---

## ⚙️ Build

```bash
mkdir -p ros-ai/src && cd ros-ai/src
git clone --recurse-submodules https://github.com/ros-ai/ros2_whisper.git
cd ..
colcon build --symlink-install --cmake-args -DGGML_CUDA=On --no-warn-unused-cli
```

---

## 🚀 Demos

### Whisper On Key

Launch the full pipeline:

```bash
ros2 launch whisper_bringup bringup.launch.py
```

Trigger inference with the space bar:

```bash
ros2 run whisper_demos whisper_on_key
```

### Stream Transcription

Launch pipeline:

```bash
ros2 launch whisper_bringup bringup.launch.py
```

Then run stream node:

```bash
ros2 run whisper_demos stream
```

---

## 🔧 Parameters

Enable/disable inference dynamically:

```bash
ros2 param set /whisper/inference active true  # or false
```

When `active` is false, audio is still buffered but not processed.

---

## 🎯 Actions and Topics

### Action: `/inference` ([Inference.action](whisper_idl/action/Inference.action))

- **Goal**: Run inference for a given time.
- **Feedback**: Live updating transcript.
- **Result**: Complete transcript (stale + active).

### Published Topics:

- `/whisper/transcript_stream` → [`AudioTranscript.msg`](whisper_idl/msg/AudioTranscript.msg)
- `/whisper/tokens` → [`WhisperTokens.msg`](whisper_idl/msg/WhisperTokens.msg)

---

## 🧩 Config File

Configure parameters in:

```bash
whisper_server/config/whisper.yaml
```

---

## 🧰 Troubleshooting

- Whisper inference time and tuning: [ggml issue #10](https://github.com/ggerganov/whisper.cpp/issues/10#issuecomment-1302462960)

---

## 📜 License

Licensed under MIT. See [LICENSE](./LICENSE).
