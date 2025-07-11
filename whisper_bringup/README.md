
# 🗣️ whisper_bringup

`whisper_bringup` is a ROS 2 package that provides launch configurations for setting up and running an end-to-end real-time speech recognition pipeline. It integrates audio input, whisper inference, and transcript generation using composable nodes.

---

## 📁 Project Structure

```
whisper_bringup/
├── CMakeLists.txt
├── launch/
│   ├── bringup.launch.py   # Full pipeline: audio_listener + whisper + transcript_manager
│   └── replay.launch.py    # Launch whisper + transcript_manager only (no audio capture)
├── package.xml
├── LICENSE
└── README.md
```

---

## 🚀 Launch Files

### `bringup.launch.py`
Launches the full pipeline:
- `audio_listener`: Captures microphone audio and publishes it.
- `whisper::Inference`: Runs whisper inference using `whisper_server`.
- `transcript_manager`: Merges and refines the transcribed results.

#### Launch arguments:
| Name              | Default | Description                                   |
|-------------------|---------|-----------------------------------------------|
| `active`          | `"true"`| Whether to activate whisper node immediately  |
| `device_index`    | `-1`    | Audio input device index (from PyAudio list)  |
| `rate`            | `16000` | Sample rate in Hz                             |
| `frames_per_buffer` | `1000` | Audio chunk size per read                    |

### `replay.launch.py`
Launches whisper and transcript manager nodes only. Useful when audio is being published from a bag file or simulator.

#### Launch arguments:
| Name     | Default | Description                              |
|----------|---------|------------------------------------------|
| `active` | `"true"`| Whether to activate whisper node         |

---

## 🧠 Dependencies

As listed in `package.xml`:

- `rclcpp_components`
- `whisper_server`
- `whisper_idl`
- `whisper_util`
- `whisper_cpp_vendor`

---

## ⚙️ Build Instructions

```bash
colcon build --packages-select whisper_bringup
source install/setup.bash
```

---

## ▶️ Example Usage

### Full Pipeline (Microphone → Transcription):
```bash
ros2 launch whisper_bringup bringup.launch.py
```

### Simulated Playback / Bag Replay:
```bash
ros2 launch whisper_bringup replay.launch.py
```

---

## 📄 License

This project is licensed under the MIT License.
