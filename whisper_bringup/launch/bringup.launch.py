import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:
    active_arg = DeclareLaunchArgument(
        'active',
        default_value="true",
        description='Start with whisper node active'
    )
    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='-1',
        description='Index of the input audio device (see PyAudio device list)'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='16000',
        description='Sample rate for audio capture'
    )
    active = LaunchConfiguration('active')
    device_index = LaunchConfiguration('device_index')
    rate = LaunchConfiguration('rate')

    ld = LaunchDescription()

    # Declare launch arguments first
    ld.add_action(active_arg)
    ld.add_action(device_index_arg)
    ld.add_action(rate_arg)

    # launch audio listener with device_index and rate as parameters
    ld.add_action(
        Node(
            package="audio_listener",
            executable="audio_listener",
            output="screen",
            parameters=[{
                'device_index': device_index,
                'rate': rate
            }]
        )
    )

    # launch whisper
    whisper_config = os.path.join(
        get_package_share_directory("whisper_server"), "config", "whisper.yaml"
    )

    container = ComposableNodeContainer(
            name='whisper_container',
            package='rclcpp_components',
            namespace='',
            executable='component_container_mt',  # Use 'component_container' for single-threaded
            output={
                    'stdout': 'screen',
                    'stderr': 'screen',
                },
            emulate_tty=True,
            composable_node_descriptions=[
                # Whisper
                ComposableNode(
                    package='whisper_server',
                    plugin='whisper::Inference',
                    name='inference',
                    namespace="whisper",
                    parameters=[whisper_config, {'active': active}],
                    remappings=[("audio", "/audio_listener/audio")],
                ),
                # Transcript manager
                ComposableNode(
                    package='transcript_manager',
                    plugin='whisper::TranscriptManager',
                    name='transcript_manager',
                    namespace="whisper",
                ),
            ],
        )
    ld.add_action(container)
    return ld
