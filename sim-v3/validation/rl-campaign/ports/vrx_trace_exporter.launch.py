from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path

def generate_launch_description():
    script=Path(__file__).with_name('vrx_trace_exporter.py')
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='/tmp/vrx-trace-samples.jsonl'),
        ExecuteProcess(cmd=['python3', str(script), '--ros-args', '-p', ['output:=', LaunchConfiguration('output')]], output='screen')
    ])
