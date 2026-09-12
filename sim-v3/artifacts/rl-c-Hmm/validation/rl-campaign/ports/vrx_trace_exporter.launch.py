from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path

def generate_launch_description():
    script=Path(__file__).with_name('vrx_trace_exporter.py')
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='/tmp/vrx-trace-samples.jsonl'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('initial_n_m', default_value='10000'),
        DeclareLaunchArgument('initial_e_m', default_value='10000'),
        ExecuteProcess(cmd=['python3', str(script), '--ros-args',
            '-p', ['output:=', LaunchConfiguration('output')],
            '-p', ['seed:=', LaunchConfiguration('seed')],
            '-p', ['initial_n_m:=', LaunchConfiguration('initial_n_m')],
            '-p', ['initial_e_m:=', LaunchConfiguration('initial_e_m')]], output='screen')
    ])
