
# 📦 whisper_idl

`whisper_idl` is a ROS 2 interface definition package that defines the core **message** and **action** types used throughout the whisper-based speech recognition pipeline.

This package includes IDL files (`.msg`, `.action`) that enable communication between components like `audio_listener`, `whisper_server`, and `transcript_manager`.

---

## 📁 Project Structure

```
whisper_idl/
├── action/
│   └── Inference.action         # Action definition for triggering whisper inference
├── msg/
│   ├── AudioTranscript.msg      # Final transcript message with timestamps and word data
│   └── WhisperTokens.msg        # Tokenized input message from audio stream
├── CMakeLists.txt
├── package.xml
├── LICENSE
```

---

## 🧾 Defined Interfaces

### `WhisperTokens.msg`

- Contains raw tokens, probabilities, timestamps, and segment start indices.
- Used to transmit recognized word tokens from audio input processing.

### `AudioTranscript.msg`

- Contains finalized and active transcriptions with:
  - Words
  - Confidence probabilities
  - Occurrence counts
  - Segment start times and durations

### `Inference.action`

Defines the speech recognition action interface:

- **Goal**: duration to run inference (max duration)
- **Feedback**: streaming transcriptions (`batch_idx`, partial text)
- **Result**: finalized transcriptions after inference ends or is canceled

---

## 🛠️ Build Integration

The following CMake logic auto-generates ROS 2 code from `.msg` and `.action` files:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Inference.action"
  "msg/WhisperTokens.msg"
  "msg/AudioTranscript.msg"
  DEPENDENCIES
    builtin_interfaces
)
```

---

## ⚙️ Dependencies

- `builtin_interfaces`
- `rosidl_default_generators`
- `rosidl_default_runtime`

---

## 🚀 Build Instructions

```bash
colcon build --packages-select whisper_idl
source install/setup.bash
```

Then, import the generated messages and actions in your Python or C++ nodes.

---

## 📄 License

This project is licensed under the MIT License.
