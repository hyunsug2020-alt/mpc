#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ==============================================================================
# [1] 차량별 경로(Node Sequence) 데이터베이스 (CAV용)
# ==============================================================================
CAV_PATH_SETTINGS = [
    [21, 51, 46, 40, 43, 9, 56, 59, 18, 21],  # CAV 1
    [60, 52, 24, 37, 39, 49, 55, 12, 15, 60], # CAV 2
    [19, 22, 25, 36, 38, 48, 58, 19],         # CAV 3
    [16, 61, 47, 41, 42, 8, 11, 16],          # CAV 4
]

def load_yaml_file(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        return {}

def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")
    
    # 설정 파일 경로
    config_file = os.path.join(pkg_dir, "config", "cav_config.yaml")
    # RViz 설정 파일 경로 (bisa.rviz 파일이 해당 위치에 있어야 함)
    rviz_config = os.path.join(pkg_dir, "rviz", "bisa.rviz")

    # 1. YAML 파일 로드
    full_config = load_yaml_file(config_file)

    # 파라미터 추출 로직
    try:
        ros_params_dict = full_config["/**"]["ros__parameters"]
    except KeyError:
        ros_params_dict = {"cav_ids": [1, 2, 3, 4]}

    hv_settings = full_config.get("hv_settings", [])
    
    nodes = []

    # ---------------------------------------------------------
    # [추가됨] RViz2 실행 노드
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],  # 설정된 .rviz 파일 로드
            output='screen',
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )

    # ---------------------------------------------------------
    # 2. 공통 노드 (HDMap Visualizer)
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package="bisa",
            executable="hdmap_visualizer.py",
            name="hdmap_visualizer",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )

    # ---------------------------------------------------------
    # 3. HV 차량 자동 생성
    # ---------------------------------------------------------
    for hv in hv_settings:
        hv_id = hv["id"]
        hv_path = hv["node_sequence"]

        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_hv{hv_id}",
                output="screen",
                parameters=[
                    {"cav_id": hv_id, "node_sequence": hv_path, "rviz_slot": -1}
                ],
                remappings=[("/user_global_path", f"/hv{hv_id}/global_path")],
                additional_env={"ROS_DOMAIN_ID": str(hv_id)},
            )
        )

    # ---------------------------------------------------------
    # 4. 동적 CAV 차량 노드 생성
    # ---------------------------------------------------------
    active_ids = ros_params_dict.get("cav_ids", [1, 2, 3, 4])

    # CAV 경로 설정 데이터 수에 맞춰 반복 (최대 4대)
    loop_count = min(len(active_ids), len(CAV_PATH_SETTINGS))

    for index in range(loop_count):
        cav_id = active_ids[index]
        node_seq = CAV_PATH_SETTINGS[index]
        
        id_str = f"{cav_id:02d}"    # 예: "01"
        slot_str = f"{index + 1:02d}" # 예: "01"
        rviz_slot = index           # RViz 시각화 슬롯 (0~3)
        cav_prefix = f"/cav{id_str}"

        # YAML에서 CAV ID 기반 섹션 로드. 없으면 slot 기반 섹션으로 fallback.
        yaml_section_name = f"mpc_tracker_cav{id_str}"
        yaml_section_fallback = f"mpc_tracker_cav{slot_str}"
        node_params = {}
        if yaml_section_name in full_config:
            node_params = full_config[yaml_section_name].get("ros__parameters", {})
        elif yaml_section_fallback in full_config:
            node_params = full_config[yaml_section_fallback].get("ros__parameters", {})

        # (A) Global Path Publisher
        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    {
                        "cav_id": cav_id,
                        "node_sequence": node_seq,
                        "rviz_slot": rviz_slot,
                    }
                ],
                remappings=[
                    ("/user_global_path", f"{cav_prefix}/global_path"),
                    (f"/viz/slot{rviz_slot}/global_path", f"{cav_prefix}/viz/global_path"),
                ],
                additional_env={"ROS_DOMAIN_ID": str(cav_id)},
            )
        )

        # (B) Local Path Publisher
        nodes.append(
            Node(
                package="bisa",
                executable="local_path_pub_cpp",
                name=f"local_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    ros_params_dict, # 공통 파라미터
                    {
                        "target_cav_id": cav_id,
                        "rviz_slot": rviz_slot,
                    },
                ],
                remappings=[
                    (f"/user_global_path_cav{id_str}", f"{cav_prefix}/global_path"),
                    (f"/CAV_{id_str}", f"{cav_prefix}/ego_pose"),
                    (f"/local_path_cav{id_str}", f"{cav_prefix}/local_path"),
                    (f"/car_marker_{id_str}", f"{cav_prefix}/car_marker"),
                    (f"/lap_info_cav{id_str}", f"{cav_prefix}/lap_info"),
                    (f"/viz/slot{rviz_slot}/local_path", f"{cav_prefix}/viz/local_path"),
                    (f"/viz/slot{rviz_slot}/car_marker", f"{cav_prefix}/viz/car_marker"),
                ],
                additional_env={"ROS_DOMAIN_ID": str(cav_id)},
            )
        )

        # (C) MPC Path Tracker
        nodes.append(
            Node(
                package="bisa",
                executable="mpc_path_tracker_cpp",
                name=f"mpc_tracker_cav{id_str}",
                output="screen",
                parameters=[node_params, {"target_cav_id": cav_id}],
                remappings=[
                    ("/local_path", f"{cav_prefix}/local_path"),
                    ("/Ego_pose", f"{cav_prefix}/ego_pose"),
                    ("/Accel", f"{cav_prefix}/accel_cmd"),
                    ("/Accel_raw", f"{cav_prefix}/accel_raw"),
                    ("/mpc_predicted_path", f"{cav_prefix}/mpc_predicted_path"),
                    ("/mpc_performance", f"{cav_prefix}/mpc_performance"),
                ],
                additional_env={"ROS_DOMAIN_ID": str(cav_id)},
            )
        )

    return LaunchDescription(nodes)
