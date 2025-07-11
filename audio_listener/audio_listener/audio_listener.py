import atexit

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from scipy.signal import resample_poly


class AudioListenerNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("channels", 1),
                ("frames_per_buffer", 1000),
                ("rate", 16000),
                ("device_index", -1),
            ],
        )

        self.channels_ = (
            self.get_parameter("channels").get_parameter_value().integer_value
        )
        self.frames_per_buffer_ = (
            self.get_parameter("frames_per_buffer").get_parameter_value().integer_value
        )
        self.input_rate_ = self.get_parameter("rate").get_parameter_value().integer_value
        self.device_index_ = (
            self.get_parameter("device_index").get_parameter_value().integer_value
        )

        self.target_rate_ = 16000  # Always publish at 16kHz
        self.resample_required_ = self.input_rate_ != self.target_rate_

        print("\n[AudioListenerNode] Starting with parameters:")
        print(f"  channels          : {self.channels_}")
        print(f"  frames_per_buffer : {self.frames_per_buffer_}")
        print(f"  input_rate        : {self.input_rate_}")
        print(f"  target_rate       : {self.target_rate_} (fixed)")
        print(f"  device_index      : {self.device_index_}")
        print(f"  resample_required : {self.resample_required_}")

        self.pyaudio_ = pyaudio.PyAudio()
        self.stream_ = self.pyaudio_.open(
            channels=self.channels_,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frames_per_buffer_,
            rate=self.input_rate_,
            input_device_index=(self.device_index_ if self.device_index_ >= 0 else None),
        )

        self.audio_publisher_ = self.create_publisher(
            Int16MultiArray, "~/audio", qos_profile=qos_profile_sensor_data
        )

        self.audio_publisher_timer_ = self.create_timer(
            float(self.frames_per_buffer_) / float(self.input_rate_),
            self.audio_publisher_timer_callback_,
        )

        self.buffer = []  # Buffer to accumulate raw audio samples

        atexit.register(self.cleanup_)

    def audio_publisher_timer_callback_(self) -> None:
        audio = self.stream_.read(self.frames_per_buffer_, exception_on_overflow=False)
        audio = np.frombuffer(audio, dtype=np.int16)
        self.buffer.extend(audio)

        if self.resample_required_:
            required_input_samples = int(self.target_rate_ * (self.input_rate_ / self.target_rate_))
        else:
            required_input_samples = self.target_rate_

        while len(self.buffer) >= required_input_samples:
            chunk = self.buffer[:required_input_samples]
            self.buffer = self.buffer[required_input_samples:]

            if self.resample_required_:
                # Resample from input_rate_ to 16000 using polyphase filter
                resampled = resample_poly(chunk, self.target_rate_, self.input_rate_)
                resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
                chunk = resampled
            else:
                chunk = np.array(chunk, dtype=np.int16)

            audio_msg = Int16MultiArray()
            audio_msg.data = [int(x) for x in chunk]
            audio_msg.layout.data_offset = 0
            audio_msg.layout.dim = [
                MultiArrayDimension(label="audio", size=len(chunk), stride=1)
            ]
            self.audio_publisher_.publish(audio_msg)
            self.get_logger().info(f"🔊 Published 1 sec of audio ({len(audio_msg.data)} samples)")

    def cleanup_(self):
        try:
            self.stream_.close()
        except Exception:
            pass
        try:
            self.pyaudio_.terminate()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    audio_listener = AudioListenerNode("audio_listener")
    rclpy.spin(audio_listener)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
