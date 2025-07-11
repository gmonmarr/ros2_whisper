
# 🛠️ whisper_util

`whisper_util` is a ROS 2 C++ utility library providing reusable audio processing components for use with [whisper.cpp](https://github.com/ggerganov/whisper.cpp). It includes thread-safe audio buffering, Whisper model management, and wrapper logic for performing inference.

---

## 📦 Overview

This package provides:

- `AudioRing`: a thread-safe ring buffer for streaming audio data.
- `Whisper`: a wrapper around `whisper.cpp` for transcription and token-level data.
- `ModelManager`: utility to download and manage whisper models from a remote source.
- Utility headers for time and audio conversions.

---

## 📁 Directory Structure

```
whisper_util/
├── include/whisper_util/
│   ├── audio_buffers.hpp       # Audio buffering utilities
│   ├── chrono_utils.hpp        # Time conversion helpers
│   ├── model_manager.hpp       # Whisper model download + management
│   └── whisper.hpp             # Wrapper around whisper.cpp
├── src/
│   ├── audio_buffers.cpp
│   ├── model_manager.cpp
│   └── whisper.cpp
├── CMakeLists.txt
├── package.xml
├── LICENSE
```

---

## 🧩 Key Components

### `AudioRing`
- Buffers `int16_t` audio samples in real-time.
- Keeps a consistent audio duration window.
- Offers a `peak()` method to return normalized float vectors with timestamp.

### `Whisper`
- Wraps whisper.cpp inference.
- Provides `forward()` for basic text output.
- Provides `forward_serialize()` for tokenized outputs (IDs, timestamps, text, probabilities).

### `ModelManager`
- Downloads Whisper models from a remote URL if not cached.
- Resolves full paths and checks model availability.

---

## ⚙️ Dependencies

- [whisper_cpp_vendor](https://github.com/ggerganov/whisper.cpp) (for model runtime)
- C++17
- ROS 2 (ament_cmake)

---

## 🔧 Build

```bash
colcon build --packages-select whisper_util
source install/setup.bash
```

---

## 📝 License

MIT License.
