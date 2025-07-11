
# 📜 transcript_manager

`transcript_manager` is a modular ROS 2 C++ package that receives tokenized audio transcription data (e.g. from Whisper), manages streaming updates, and merges them into consistent and refined transcripts. It uses intelligent alignment algorithms (e.g., LCS with allowed gaps) to merge overlapping segments while preserving word confidence and segment timing.

---

## 📁 Project Structure

```
transcript_manager/
├── CMakeLists.txt
├── include/
│   └── transcript_manager/
│       ├── segments.hpp
│       ├── tokens.hpp
│       ├── transcript.hpp
│       ├── transcript_manager.hpp
│       └── words.hpp
├── src/
│   ├── segments.cpp
│   ├── transcript.cpp
│   ├── transcript_algorithms.cpp
│   ├── transcript_manager.cpp
│   └── transcript_operations.cpp
├── package.xml
├── LICENSE
└── README.md
```

---

## 🚀 Main Components

### `TranscriptManager` Node (`transcript_manager.cpp`)
- ROS 2 component node that:
  - Subscribes to incoming token messages (e.g., `WhisperTokens`).
  - Runs an action server for triggering transcript inference.
  - Buffers and merges incoming data using LCS-based alignment.
  - Publishes the assembled transcript as `AudioTranscript`.

### `Transcript` Class (`transcript.cpp`, `transcript_algorithms.cpp`, `transcript_operations.cpp`)
- Central structure for managing transcript segments and their words.
- Key features:
  - LCS merging with tolerance for insertions/deletions.
  - Tracking of stale (finalized) vs. active segments.
  - Increment/decrement confidence, conflict handling, and word/segment ops.

### `Segment` and `Word` Models
- Represent time-aligned groups of words with timing and metadata.
- Punctuation, brackets, and formatting are handled intelligently.
- Supports merging, overwriting, or discarding segments and words.

---

## 🧠 Core Logic and Algorithms

- **Token Stream → Segment Structure**: Tokens are grouped into words and then into segments with proper start/end timestamps.
- **Merge Updates**: New streaming updates are merged into the current transcript using an enhanced **LCS algorithm with gap support**.
- **Operation Pipeline**:
  - INSERT, DELETE, CONFLICT, MERGE_SEG, etc.
  - Applied sequentially with safe offset handling and ID validation.
- **Confidence Handling**:
  - Words are tracked with `occurrences` and confidence scores.
  - Low-confidence segments/words can be pruned automatically.

---

## 📡 ROS Interfaces

### Subscribed Topics
- `tokens` (`WhisperTokens`)
  - Stream of tokenized words with timestamps and confidence values.

### Published Topics
- `transcript_stream` (`AudioTranscript`)
  - Structured transcript with segment boundaries, timestamps, and word confidence.

### Actions
- `inference` (`Inference`)
  - Accepts a goal with a max duration.
  - Publishes feedback with the current transcript.
  - Returns the final result when inference ends or is canceled.

---

## 🧪 Example Use Case

1. A Whisper-based audio pipeline publishes tokens.
2. `transcript_manager`:
   - Parses the tokens into segments and words.
   - Merges new segments intelligently.
   - Publishes finalized transcripts.
3. Output can be used for:
   - Subtitle generation
   - Real-time voice interfaces
   - Language learning apps

---

## ⚙️ Parameters

| Parameter           | Type | Default | Description                          |
|---------------------|------|---------|--------------------------------------|
| `allowed_lcs_gaps`  | int  | 4       | Tolerance in LCS matching for insertions/deletions |

---

## 📄 License

This project is licensed under the MIT License.
