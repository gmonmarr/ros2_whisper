
# 🎤 whisper_demos

`whisper_demos` is a ROS 2 Python package that provides interactive and visual demonstrations for real-time speech recognition using the `whisper` pipeline. These demos showcase how to trigger and visualize transcriptions from spoken input using keyboard or streamed updates.

---

## 📁 Project Structure

```
whisper_demos/
├── whisper_demos/
│   ├── __init__.py
│   ├── stream.py              # Live transcript viewer with color-coded confidence
│   └── whisper_on_key.py      # Keyboard-triggered speech-to-text demo
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
├── setup.cfg
├── LICENSE
└── resource/
    └── whisper_demos
```

---

## 🧪 Demo Scripts

### `whisper_on_key.py`

- Press `SPACE` to trigger Whisper transcription for 20 seconds.
- Press `ESC` to stop the demo and exit.
- Uses a ROS 2 action client to `/whisper/inference`.

```bash
ros2 run whisper_demos whisper_on_key
```

---

### `stream.py`

- Subscribes to `/whisper/transcript_stream`.
- Displays the full transcript in the terminal with:
  - **Timestamps**
  - **Segment durations**
  - **Color-coded word confidence**

```bash
ros2 run whisper_demos stream
```

---

## ⚙️ Parameters (for `stream.py`)

| Name       | Type  | Default | Description                                       |
|------------|-------|---------|---------------------------------------------------|
| `threshold`| float | `1.0`   | Min score (probability × occurrence) to display finalized words |

---

## 📦 Dependencies

As declared in `package.xml`:

- `audio_listener`
- `rclpy`
- `builtin_interfaces`
- `whisper_idl`
- `whisper_server`

---

## ✅ Testing

The `test/` directory includes linting and style checks:

- `flake8`, `pep257`, `copyright`

Run tests with:
```bash
colcon test --packages-select whisper_demos
```

---

## 📝 License

This project is licensed under the MIT License.
