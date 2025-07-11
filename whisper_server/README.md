
# 🧠 whisper_server

`whisper_server` is a ROS 2 C++ component that runs real-time inference using [whisper.cpp](https://github.com/ggerganov/whisper.cpp). It consumes audio data from a ROS topic, performs speech-to-text inference, and publishes tokenized results.

---

## 📦 Overview

This package registers a component node (`whisper::Inference`) that:

- Subscribes to audio input (`std_msgs/Int16MultiArray`).
- Buffers incoming audio using a ring buffer.
- Performs inference every `callback_ms` milliseconds using whisper.cpp.
- Publishes recognized tokens via `WhisperTokens` messages.

---

## 📁 Project Structure

```
whisper_server/
├── config/whisper.yaml                # Parameter configuration file
├── include/whisper_server/inference.hpp  # Component class definition
├── src/inference.cpp                 # Main component implementation
├── CMakeLists.txt
├── package.xml
├── LICENSE
```

---

## 🧪 Subscriptions & Publications

- **Subscribes to**: `/audio` (`std_msgs/Int16MultiArray`)
- **Publishes to**: `/tokens` (`whisper_idl/msg/WhisperTokens`)

---

## ⚙️ Parameters

| Parameter               | Type     | Default    | Description |
|------------------------|----------|------------|-------------|
| `buffer_capacity`      | `int`    | `2`        | Buffer duration in seconds |
| `callback_ms`          | `int`    | `200`      | Timer period for inference |
| `active`               | `bool`   | `false`    | Whether inference is active |
| `model_name`           | `string` | `"base.en"`| Whisper model name |
| `wparams.language`     | `string` | `"en"`     | Inference language |
| `wparams.n_threads`    | `int`    | `4`        | CPU thread count |
| `wparams.print_progress` | `bool` | `false`    | Print inference progress |
| `cparams.use_gpu`      | `bool`   | `true`     | Whether to use GPU |
| `cparams.gpu_device`   | `int`    | `0`        | GPU device index |
| `cparams.flash_attn`   | `bool`   | `true`     | Enable FlashAttention optimization |

---

## 🚀 Launch

To use this component in your own launch files, load it via a container or node composition:

```bash
ros2 run whisper_server whisper
```

Or via a launch file that loads the component dynamically.

---

## 🧠 Component Registration

The component is registered using:

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(whisper::Inference)
```

Allowing it to be dynamically loaded into a container.

---

## 🔧 Build Instructions

```bash
colcon build --packages-select whisper_server
source install/setup.bash
```

---

## 🧼 Runtime Behavior

- Automatically downloads and initializes the specified whisper model if not available.
- Buffers audio data in memory and runs inference in fixed intervals.
- Reports inference duration and warns if audio is skipped due to slow performance.

---

## 📝 License

This project is licensed under the MIT License.
