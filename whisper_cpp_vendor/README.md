
# 🧱 whisper_cpp_vendor

`whisper_cpp_vendor` is a ROS 2 vendor package that automatically fetches and builds the [whisper.cpp](https://github.com/ggerganov/whisper.cpp) library as a CMake dependency. This package enables seamless integration of `whisper.cpp` into your ROS 2 workspace.

---

## 📦 Purpose

- Provides a clean and isolated integration of `whisper.cpp` as a system dependency.
- Makes its headers and compiled libraries available for downstream packages like `whisper_server`.
- Avoids manual installation or versioning issues of external non-ROS libraries.

---

## 🛠️ How It Works

- Uses CMake's `FetchContent` to download `whisper.cpp` at a specified version.
- Applies minor fixes to set target properties (e.g., include directories, standard).
- Installs headers and libraries (`ggml`, `whisper`) for other packages to use.

---

## 📁 Project Structure

```
whisper_cpp_vendor/
├── CMakeLists.txt       # Fetch + build + export whisper.cpp
├── package.xml          # ROS 2 package metadata
├── LICENSE              # MIT License
└── README.md
```

---

## ⚙️ Configurable Whisper Version

By default, the following version is used:

- **v1.7.2**

You can override this by setting cache variables when building:

```bash
colcon build --packages-select whisper_cpp_vendor   --cmake-args -DWHISPER_VERSION_MAJOR=1 -DWHISPER_VERSION_MINOR=7 -DWHISPER_VERSION_PATCH=3
```

---

## 🚀 Build Instructions

```bash
colcon build --packages-select whisper_cpp_vendor
source install/setup.bash
```

---

## 🔁 Exported Targets

- **Target name**: `whisper`
- **Include paths**:
  - `${whisper_SOURCE_DIR}/include`
  - `${whisper_SOURCE_DIR}/ggml/include`
- **Libraries**:
  - `libwhisper.so` / `libwhisper.a`
  - `libggml.so` / `libggml.a`

These are made available to downstream packages via `ament_export_targets`.

---

## 📄 License

This vendor wrapper is licensed under the MIT License.

The upstream `whisper.cpp` project is also under the MIT License.
