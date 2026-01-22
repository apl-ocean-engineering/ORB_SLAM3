#!/usr/bin/env python3

import os
import errno
import argparse
import yaml
import cv2
import numpy as np
from collections import defaultdict

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, Imu, CameraInfo
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge

CAM_FOLDER_NAME = "cam"
IMU_FOLDER_NAME = "imu"
DATA_CSV = "data.csv"
SENSOR_YAML = "sensor.yaml"
BODY_YAML = "body.yaml"

DEFAULT_IMU_SENSOR_YAML = dict(
    sensor_type="imu",
    comment="Default IMU",
    T_BS=dict(cols=4, rows=4, data=[1.0] * 16),
    rate_hz=200,
    gyroscope_noise_density=1.6968e-04,
    gyroscope_random_walk=1.9393e-05,
    accelerometer_noise_density=2.0e-3,
    accelerometer_random_walk=3.0e-3,
)


def mkdirs_without_exception(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def collect_camera_info(rosbag_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=rosbag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    camera_info_dict = {}

    print("Collecting CameraInfo messages...")
    while reader.has_next():
        topic, data, t = reader.read_next()
        if "camera_info" in topic.lower():
            camera_info_dict[topic] = deserialize_message(data, CameraInfo)
            print(f"  Found CameraInfo: {topic}")

    return camera_info_dict


def auto_select_camera_topics(topic_types):
    groups = defaultdict(list)
    for topic, msg_type in topic_types.items():
        if msg_type == "sensor_msgs/msg/Image":
            if "/left/" in topic.lower():
                groups["left"].append(topic)
            elif "/right/" in topic.lower():
                groups["right"].append(topic)
            else:
                groups["other"].append(topic)

    selected = []
    for cam, topics in sorted(groups.items()):
        # Prefer image_rect over image_raw
        if any("image_rect" in t and "compressed" not in t.lower() for t in topics):
            selected.append(
                [
                    t
                    for t in topics
                    if "image_rect" in t and "compressed" not in t.lower()
                ][0]
            )
        elif any("image_raw" in t for t in topics):
            selected.append([t for t in topics if "image_raw" in t][0])
        elif topics:
            selected.append(topics[0])

    return selected


def setup_dataset_dirs(
    camera_topics, imu_topics, camera_info_dict, output_path, rosbag_path
):
    dirname = os.path.split(rosbag_path)[-1].split(".", 1)[0] + "/mav0"
    base_path = os.path.join(output_path, dirname)
    mkdirs_without_exception(base_path)

    # Map image topics to CameraInfo topics
    cam_info_mapping = {}
    for cam_topic in camera_topics:
        # Try to find matching camera_info topic
        potential_info_topic = cam_topic.replace("/image_raw", "/camera_info").replace(
            "/image_rect", "/camera_info"
        )
        if potential_info_topic in camera_info_dict:
            cam_info_mapping[cam_topic] = potential_info_topic
        else:
            # Fallback to simple name matching
            if "left" in cam_topic.lower():
                cam_info_mapping[cam_topic] = "/left/camera_info"
            elif "right" in cam_topic.lower():
                cam_info_mapping[cam_topic] = "/right/camera_info"
            else:
                cam_info_mapping[cam_topic] = None

    print(f"\nCamera topic mapping:")
    for img_topic, info_topic in cam_info_mapping.items():
        print(f"  {img_topic} -> {info_topic}")

    # Create camera folders
    cam_folder_paths = []
    for i, cam_topic in enumerate(camera_topics):
        cam_folder = os.path.join(base_path, CAM_FOLDER_NAME + str(i))
        mkdirs_without_exception(cam_folder)
        mkdirs_without_exception(os.path.join(cam_folder, "data"))

        with open(os.path.join(cam_folder, DATA_CSV), "w+") as f:
            f.write("#timestamp [ns]")

        # Get camera info from CameraInfo message
        msg = camera_info_dict.get(cam_info_mapping.get(cam_topic, None), None)
        if msg:
            intrinsics = [
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            ]
            resolution = [msg.width, msg.height]
            distortion = list(msg.d) if msg.d else [0.0] * 4
            distortion_model = (
                msg.distortion_model if msg.distortion_model else "radtan"
            )

            cam_yaml = dict(
                sensor_type="camera",
                comment=f"{cam_topic}",
                T_BS=dict(cols=4, rows=4, data=[1.0] * 16),
                rate_hz=10,
                resolution=resolution,
                camera_model="pinhole",
                intrinsics=intrinsics,
                distortion_model=distortion_model,
                distortion_coefficients=distortion,
            )
        else:
            print(f"WARNING: No CameraInfo for {cam_topic}, using minimal defaults")
            cam_yaml = dict(
                sensor_type="camera",
                comment=f"{cam_topic} (no CameraInfo found)",
                T_BS=dict(cols=4, rows=4, data=[1.0] * 16),
                rate_hz=10,
                resolution=[640, 480],
                camera_model="pinhole",
                intrinsics=[458.654, 457.296, 320.0, 240.0],
                distortion_model="radtan",
                distortion_coefficients=[0.0, 0.0, 0.0, 0.0],
            )

        with open(os.path.join(cam_folder, SENSOR_YAML), "w+") as f:
            f.write("%YAML:1.0\n")
            yaml.dump(cam_yaml, f, default_flow_style=True)

        print(f"\nCreated {cam_folder}/sensor.yaml:")
        print(f"  Resolution: {cam_yaml['resolution']}")
        print(f"  Intrinsics: {cam_yaml['intrinsics']}")
        print(f"  Distortion: {cam_yaml['distortion_coefficients']}")

        cam_folder_paths.append(cam_folder)

    # IMU folders
    imu_folder_paths = []
    for i, imu_topic in enumerate(imu_topics):
        imu_folder = os.path.join(base_path, IMU_FOLDER_NAME + str(i))
        mkdirs_without_exception(imu_folder)
        with open(os.path.join(imu_folder, DATA_CSV), "w+") as f:
            f.write(
                "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
            )
        imu_yaml = DEFAULT_IMU_SENSOR_YAML.copy()
        imu_yaml["comment"] = imu_topic
        with open(os.path.join(imu_folder, SENSOR_YAML), "w+") as f:
            f.write("%YAML:1.0\n")
            yaml.dump(imu_yaml, f, default_flow_style=True)
        imu_folder_paths.append(imu_folder)

    # body.yaml
    with open(os.path.join(base_path, BODY_YAML), "w+") as f:
        f.write("%YAML:1.0\n")
        body_yaml = dict(
            comment=f"Automatically generated dataset using Rosbag2Euroc from rosbag: {rosbag_path}"
        )
        yaml.dump(body_yaml, f, default_flow_style=True)

    return cam_folder_paths, imu_folder_paths, base_path


def rosbag2_to_euroc(
    rosbag_path,
    output_path,
    camera_topics_arg=None,
    imu_topics_arg=None,
    auto_select=True,
    sync_tolerance_ns=5000000,
):
    if not os.path.exists(rosbag_path):
        print(f"ERROR: {rosbag_path} does not exist.")
        return

    print(f"\n{'=' * 60}")
    print(f"ROS2 Bag to EuRoC Converter")
    print(f"{'=' * 60}")
    print(f"Input bag: {rosbag_path}")
    print(f"Output path: {output_path}")
    print(f"Sync tolerance: {sync_tolerance_ns}ns ({sync_tolerance_ns / 1e6:.1f}ms)")
    print(f"{'=' * 60}\n")

    camera_info_dict = collect_camera_info(rosbag_path)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=rosbag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Select camera topics
    if camera_topics_arg:
        camera_topics = [t for t in camera_topics_arg if t in topic_types]
        print(f"Using specified camera topics: {camera_topics}")
    elif auto_select:
        camera_topics = auto_select_camera_topics(topic_types)
        print(f"Auto-selected camera topics: {camera_topics}")
    else:
        camera_topics = [
            t for t, ty in topic_types.items() if ty == "sensor_msgs/msg/Image"
        ]
        print(f"Using all image topics: {camera_topics}")

    # Select IMU topics
    if imu_topics_arg:
        imu_topics = [t for t in imu_topics_arg if t in topic_types]
    else:
        imu_topics = [t for t, ty in topic_types.items() if ty == "sensor_msgs/msg/Imu"]

    print(f"IMU topics: {imu_topics}\n")

    cam_folder_paths, imu_folder_paths, base_path = setup_dataset_dirs(
        camera_topics, imu_topics, camera_info_dict, output_path, rosbag_path
    )

    bridge = CvBridge()

    # For stereo synchronization
    if len(camera_topics) == 2:
        print(f"\n{'=' * 60}")
        print("Stereo Synchronization Mode")
        print(
            f"Sync tolerance: {sync_tolerance_ns}ns ({sync_tolerance_ns / 1e6:.1f}ms)"
        )
        print(f"{'=' * 60}\n")

        # Collect all stereo pairs first
        left_images = {}
        right_images = {}

        print("Pass 1: Collecting all image messages...")
        reader_pass1 = SequentialReader()
        reader_pass1.open(storage_options, converter_options)

        while reader_pass1.has_next():
            topic, data, t = reader_pass1.read_next()
            if topic == camera_topics[0]:  # Left camera
                msg = deserialize_message(data, Image)
                timestamp_ns = (
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                )
                left_images[timestamp_ns] = (msg, topic)
            elif topic == camera_topics[1]:  # Right camera
                msg = deserialize_message(data, Image)
                timestamp_ns = (
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                )
                right_images[timestamp_ns] = (msg, topic)

        print(f"  Found {len(left_images)} left images")
        print(f"  Found {len(right_images)} right images")

        # Match left and right images
        print("\nPass 2: Matching stereo pairs...")
        left_timestamps = sorted(left_images.keys())
        right_timestamps = sorted(right_images.keys())

        matched_pairs = []

        for left_ts in left_timestamps:
            # Find closest right timestamp
            closest_right_ts = min(right_timestamps, key=lambda x: abs(x - left_ts))
            time_diff = abs(closest_right_ts - left_ts)

            if time_diff <= sync_tolerance_ns:
                matched_pairs.append(
                    (
                        left_ts,
                        closest_right_ts,
                        left_images[left_ts],
                        right_images[closest_right_ts],
                    )
                )

        print(f"  Matched pairs: {len(matched_pairs)}")
        if len(matched_pairs) > 0:
            avg_diff = (
                sum([abs(l - r) for l, r, _, _ in matched_pairs])
                / len(matched_pairs)
                / 1e6
            )
            print(f"  Average time difference: {avg_diff:.2f}ms")

        # Process matched pairs
        print(f"\nPass 3: Processing synchronized stereo pairs...")

        # Print first few timestamps for verification
        if len(matched_pairs) > 0:
            print(f"\nFirst 3 timestamps (for verification):")
            for i in range(min(3, len(matched_pairs))):
                left_ts, right_ts, _, _ = matched_pairs[i]
                print(
                    f"  Pair {i}: left={left_ts}ns, right={right_ts}ns, diff={abs(left_ts - right_ts) / 1e6:.2f}ms"
                )

        for idx, (left_ts, right_ts, left_data, right_data) in enumerate(matched_pairs):
            left_msg, left_topic = left_data
            right_msg, right_topic = right_data

            # Use left timestamp as reference
            timestamp_ns = left_ts
            filename = f"{timestamp_ns}.png"

            # Process left image
            cv_image_left = bridge.imgmsg_to_cv2(
                left_msg, desired_encoding="passthrough"
            )
            # Convert to BGR if needed
            if len(cv_image_left.shape) == 2:  # Grayscale
                cv_image_left = cv2.cvtColor(cv_image_left, cv2.COLOR_GRAY2BGR)
            elif cv_image_left.shape[2] == 3 and left_msg.encoding == "rgb8":
                cv_image_left = cv2.cvtColor(cv_image_left, cv2.COLOR_RGB2BGR)

            # Process right image
            cv_image_right = bridge.imgmsg_to_cv2(
                right_msg, desired_encoding="passthrough"
            )
            # Convert to BGR if needed
            if len(cv_image_right.shape) == 2:  # Grayscale
                cv_image_right = cv2.cvtColor(cv_image_right, cv2.COLOR_GRAY2BGR)
            elif cv_image_right.shape[2] == 3 and right_msg.encoding == "rgb8":
                cv_image_right = cv2.cvtColor(cv_image_right, cv2.COLOR_RGB2BGR)

            # Save both images with same timestamp
            cv2.imwrite(
                os.path.join(cam_folder_paths[0], "data", filename), cv_image_left
            )
            cv2.imwrite(
                os.path.join(cam_folder_paths[1], "data", filename), cv_image_right
            )

            # Write timestamp only (not filename) for ORB-SLAM3 compatibility
            with open(os.path.join(cam_folder_paths[0], DATA_CSV), "a") as f:
                f.write(f"\n{timestamp_ns}")
            with open(os.path.join(cam_folder_paths[1], DATA_CSV), "a") as f:
                f.write(f"\n{timestamp_ns}")

            if (idx + 1) % 100 == 0:
                print(f"  Processed {idx + 1}/{len(matched_pairs)} stereo pairs...")

        img_count = len(matched_pairs) * 2

        # Process IMU separately
        print("\nProcessing IMU data...")
        reader_imu = SequentialReader()
        reader_imu.open(storage_options, converter_options)
        imu_count = 0

        while reader_imu.has_next():
            topic, data, t = reader_imu.read_next()
            if topic in imu_topics:
                msg = deserialize_message(data, Imu)
                imu_idx = imu_topics.index(topic)
                timestamp_ns = (
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                )
                with open(os.path.join(imu_folder_paths[imu_idx], DATA_CSV), "a") as f:
                    f.write(
                        f"\n{timestamp_ns},{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}"
                    )
                imu_count += 1

        msg_count = img_count + imu_count

    else:
        # Original non-synchronized processing for single camera or >2 cameras
        msg_count = 0
        img_count = 0
        imu_count = 0

        print(f"\n{'=' * 60}")
        print("Processing messages...")
        print(f"{'=' * 60}\n")

        while reader.has_next():
            topic, data, t = reader.read_next()

            if topic in camera_topics:
                msg = deserialize_message(data, Image)
                timestamp_ns = (
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                )
                filename = f"{timestamp_ns}.png"
                cam_idx = camera_topics.index(topic)

                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                # Convert to BGR if needed
                if len(cv_image.shape) == 2:  # Grayscale
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                elif cv_image.shape[2] == 3 and msg.encoding == "rgb8":
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

                cv2.imwrite(
                    os.path.join(cam_folder_paths[cam_idx], "data", filename), cv_image
                )

                # Write timestamp only (not filename) for ORB-SLAM3 compatibility
                with open(os.path.join(cam_folder_paths[cam_idx], DATA_CSV), "a") as f:
                    f.write(f"\n{timestamp_ns}")

                img_count += 1
                msg_count += 1

                if img_count % 100 == 0:
                    print(f"  Processed {img_count} images...")

            elif topic in imu_topics:
                msg = deserialize_message(data, Imu)
                imu_idx = imu_topics.index(topic)
                timestamp_ns = (
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                )
                with open(os.path.join(imu_folder_paths[imu_idx], DATA_CSV), "a") as f:
                    f.write(
                        f"\n{timestamp_ns},{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}"
                    )

                imu_count += 1
                msg_count += 1

    print(f"\n{'=' * 60}")
    print("Conversion Summary")
    print(f"{'=' * 60}")
    print(f"Total messages processed: {msg_count}")
    print(f"Images saved: {img_count}")
    print(f"IMU messages saved: {imu_count}")
    print(f"Output directory: {base_path}")
    print(f"{'=' * 60}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert ROS2 bag to EuRoC format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-select best topics
  %(prog)s /path/to/bag

  # Specify camera topics
  %(prog)s /path/to/bag --camera-topics /left/image_rect /right/image_rect

  # Custom sync tolerance for stereo
  %(prog)s /path/to/bag --sync-tolerance 10
        """,
    )
    parser.add_argument("rosbag_path", help="Path to ROS2 bag file")
    parser.add_argument(
        "-o", "--output_path", default="./", help="Output directory (default: ./)"
    )
    parser.add_argument(
        "--camera-topics", nargs="+", help="Specific camera topics to use"
    )
    parser.add_argument("--imu-topics", nargs="+", help="Specific IMU topics to use")
    parser.add_argument(
        "--no-auto-select",
        action="store_true",
        help="Disable automatic topic selection",
    )
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=5.0,
        help="Stereo sync tolerance in milliseconds (default: 5ms)",
    )
    args = parser.parse_args()

    rosbag2_to_euroc(
        args.rosbag_path,
        args.output_path,
        camera_topics_arg=args.camera_topics,
        imu_topics_arg=args.imu_topics,
        auto_select=not args.no_auto_select,
        sync_tolerance_ns=int(args.sync_tolerance * 1e6),
    )
