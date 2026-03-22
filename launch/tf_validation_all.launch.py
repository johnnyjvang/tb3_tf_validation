#!/usr/bin/env python3

"""
tf_validation_all.launch.py

Runs TurtleBot3 TF validation tests sequentially
and prints a summary report at the end.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    reset_results = Node(
        package='tb3_tf_validation',
        executable='reset_results',
        name='reset_results',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_tree_check = Node(
        package='tb3_tf_validation',
        executable='tf_tree_check',
        name='tf_tree_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_static_check = Node(
        package='tb3_tf_validation',
        executable='tf_static_check',
        name='tf_static_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_dynamic_check = Node(
        package='tb3_tf_validation',
        executable='tf_dynamic_check',
        name='tf_dynamic_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_rate_check = Node(
        package='tb3_tf_validation',
        executable='tf_rate_check',
        name='tf_rate_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_delay_check = Node(
        package='tb3_tf_validation',
        executable='tf_delay_check',
        name='tf_delay_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_lookup_test = Node(
        package='tb3_tf_validation',
        executable='tf_lookup_test',
        name='tf_lookup_test',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_motion_consistency = Node(
        package='tb3_tf_validation',
        executable='tf_motion_consistency',
        name='tf_motion_consistency',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    summary_report = Node(
        package='tb3_tf_validation',
        executable='summary_report',
        name='summary_report',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        LogInfo(msg='========================================'),
        LogInfo(msg='Starting TurtleBot3 TF Validation Suite'),
        LogInfo(msg='========================================'),

        reset_results,

        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_results,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='reset_results complete -> starting tf_tree_check'),
                    LogInfo(msg='----------------------------------------'),
                    tf_tree_check,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_tree_check,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_tree_check complete -> starting tf_static_check'),
                    LogInfo(msg='----------------------------------------'),
                    tf_static_check,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_static_check,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_static_check complete -> starting tf_dynamic_check'),
                    LogInfo(msg='----------------------------------------'),
                    tf_dynamic_check,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_dynamic_check,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_dynamic_check complete -> starting tf_rate_check'),
                    LogInfo(msg='----------------------------------------'),
                    tf_rate_check,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_rate_check,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_rate_check complete -> starting tf_delay_check'),
                    LogInfo(msg='----------------------------------------'),
                    tf_delay_check,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_delay_check,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_delay_check complete -> starting tf_lookup_test'),
                    LogInfo(msg='----------------------------------------'),
                    tf_lookup_test,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_lookup_test,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_lookup_test complete -> starting tf_motion_consistency'),
                    LogInfo(msg='----------------------------------------'),
                    tf_motion_consistency,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_motion_consistency,
                on_exit=[
                    LogInfo(msg='----------------------------------------'),
                    LogInfo(msg='tf_motion_consistency complete -> starting summary_report'),
                    LogInfo(msg='----------------------------------------'),
                    summary_report,
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=summary_report,
                on_exit=[
                    LogInfo(msg='========================================'),
                    LogInfo(msg='TurtleBot3 TF Validation Suite Complete'),
                    LogInfo(msg='========================================'),
                ]
            )
        ),
    ])