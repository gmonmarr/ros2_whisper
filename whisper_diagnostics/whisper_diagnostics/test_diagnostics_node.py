import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from whisper_idl.msg import WhisperTokens, AudioTranscript
from rcl_interfaces.srv import GetParameters

import time

class WhisperDiagnostics(Node):
    def __init__(self):
        super().__init__('whisper_diagnostics')
        self.audio_msgs = 0
        self.tokens_msgs = 0
        self.transcript_msgs = 0
        self.inference_active = None

        self.audio_sub = self.create_subscription(
            Int16MultiArray,
            '/audio_listener/audio',
            self.audio_cb,
            10
        )
        self.tokens_sub = self.create_subscription(
            WhisperTokens,
            '/whisper/tokens',
            self.tokens_cb,
            10
        )
        self.transcript_sub = self.create_subscription(
            AudioTranscript,
            '/whisper/transcript_stream',
            self.transcript_cb,
            10
        )

        self.start_time = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(5.0, self.report_summary)
        self.called_once = False

    def audio_cb(self, msg):
        self.audio_msgs += 1

    def tokens_cb(self, msg):
        self.tokens_msgs += 1

    def transcript_cb(self, msg):
        self.transcript_msgs += 1

    def report_summary(self):
        self.get_logger().info("----- Diagnostic Summary -----")

        # Audio
        if self.audio_msgs > 0:
            self.get_logger().info(f"✅ Received {self.audio_msgs} /audio messages")
        else:
            self.get_logger().error("❌ No /audio messages received")

        # Tokens
        if self.tokens_msgs > 0:
            self.get_logger().info(f"✅ Received {self.tokens_msgs} /whisper/tokens messages")
        else:
            self.get_logger().warn("⚠️ No /whisper/tokens messages received")

        # Transcript
        if self.transcript_msgs > 0:
            self.get_logger().info(f"✅ Received {self.transcript_msgs} /whisper/transcript_stream messages")
        else:
            self.get_logger().warn("⚠️ No /whisper/transcript_stream messages received")

        # Inference active param
        if not self.called_once:
            self.called_once = True
            self.check_inference_status()

    def check_inference_status(self):
        client = self.create_client(GetParameters, '/whisper/inference/get_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for parameter service...")
        req = GetParameters.Request()
        req.names = ['active']
        future = client.call_async(req)

        def on_response(fut):
            try:
                active = fut.result().values[0].bool_value
                if active:
                    self.get_logger().info("✅ Inference is ACTIVE")
                else:
                    self.get_logger().warn("⚠️ Inference is INACTIVE")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to get parameter: {e}")
        future.add_done_callback(on_response)

def main():
    rclpy.init()
    node = WhisperDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
