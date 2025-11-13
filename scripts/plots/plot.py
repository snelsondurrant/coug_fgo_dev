#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 2.5 Pro), Nov 2025

"""
Analyzes ROS2 bag files to compare localization algorithm trajectories (3D)
and calculates the L2 norm for all 6 DOF (x, y, z, roll, pitch, yaw)
against a ground truth topic.

Usage:
    python3 plot.py <path_to_rosbag_directory>
"""

import argparse
import matplotlib.pyplot as plt
import numpy as np
import math
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores


def quaternion_to_rpy(orientation):
    """Converts a ROS Quaternion message to roll, pitch, yaw."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def get_odometry_data(reader, connections, typestore):
    """Extracts 6DOF odometry data (x, y, z, roll, pitch, yaw, t) from bag messages."""
    data = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": [], "t": []}
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        data["x"].append(msg.pose.pose.position.x)
        data["y"].append(msg.pose.pose.position.y)
        data["z"].append(msg.pose.pose.position.z)

        roll, pitch, yaw = quaternion_to_rpy(msg.pose.pose.orientation)
        data["roll"].append(roll)
        data["pitch"].append(pitch)
        data["yaw"].append(yaw)

        data["t"].append(timestamp)
    return data


def calculate_component_errors(truth_data, test_data):
    """
    Calculates error over time for all 6 components using interpolation.
    Renamed from calculate_rmse.
    """
    truth_t = np.array(truth_data["t"])
    test_t = np.array(test_data["t"])

    errors = {}

    # Interpolate all components
    for key in ["x", "y", "z", "roll", "pitch", "yaw"]:
        truth_interp = np.interp(test_t, truth_t, np.array(truth_data[key]))
        test_values = np.array(test_data[key])

        if key in ["roll", "pitch", "yaw"]:
            # Handle angle wrapping for error calculation
            diff = truth_interp - test_values
            error = np.arctan2(np.sin(diff), np.cos(diff))
        else:
            error = truth_interp - test_values

        errors[key] = error

    # Return timestamps in seconds relative to start
    return (test_t - truth_t[0]) / 1e9, errors


def main():
    parser = argparse.ArgumentParser(
        description="Analyze localization data from a ROS2 bag file."
    )
    parser.add_argument(
        "bag_file", type=str, help="Path to the ROS2 bag file (directory)."
    )
    args = parser.parse_args()

    odom_topics = {
        "EKF": "/odometry/global_ekf",
        "UKF": "/odometry/global_ukf",
        "FGO": "/odometry/global",
        "Truth": "/odometry/truth",
    }

    color_map = {
        "FGO": "tab:green",
        "Truth": "k",
        "EKF": "tab:red",
        "UKF": "tab:orange",
    }

    odom_data = {}
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(args.bag_file) as reader:
        connections_by_topic = {}
        for c in reader.connections:
            if c.topic not in connections_by_topic:
                connections_by_topic[c.topic] = []
            connections_by_topic[c.topic].append(c)

        for algorithm, topic in odom_topics.items():
            try:
                connections = connections_by_topic[topic]
                odom_data[algorithm] = get_odometry_data(reader, connections, typestore)
            except KeyError:
                print(f"Warning: Topic '{topic}' for {algorithm} not found in bag.")

    # Plot 3D Trajectories (each filter vs. Truth)
    if "Truth" in odom_data and odom_data["Truth"]["x"]:
        truth_data = odom_data["Truth"]

        for algorithm, data in odom_data.items():
            if algorithm == "Truth" or not data["x"]:
                continue

            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection="3d")

            ax.plot(
                truth_data["x"],
                truth_data["y"],
                truth_data["z"],
                label="Ground Truth",
                color=color_map["Truth"],
                alpha=0.8,
            )

            color = color_map.get(algorithm)
            ax.plot(data["x"], data["y"], data["z"], label=algorithm, color=color)

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.legend()
            ax.grid(True)
            print(f"Generated 3D Trajectory Plot for {algorithm}")
    else:
        print("Skipping 3D plots as 'Truth' data is missing.")

    # Plot L2 Norm Components (x, y, z, roll, pitch, yaw)
    if "Truth" in odom_data and odom_data["Truth"]["t"]:

        fig, axs = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
        ax_flat = axs.flatten()

        components = ["x", "roll", "y", "pitch", "z", "yaw"]
        titles = ["X Error", "Roll Error", "Y Error", "Pitch Error", "Z Error", "Yaw Error"]
        units = ["m", "rad", "m", "rad", "m", "rad"]

        for algorithm, data in odom_data.items():
            if algorithm == "Truth" or not data["x"]:
                continue

            try:
                timestamps_sec, errors = calculate_component_errors(
                    odom_data["Truth"], data
                )
                plot_color = color_map.get(algorithm, "tab:gray")

                for i, comp in enumerate(components):
                    ax = ax_flat[i]
                    original_error_data = errors[comp]
                    plot_data = np.abs(original_error_data)
                    l2_norm = np.sqrt(np.mean(original_error_data**2))
                    label = f"{algorithm} (L2: {l2_norm:.4f} {units[i]})"
                    ax.plot(
                        timestamps_sec,
                        plot_data,
                        color=plot_color,
                        alpha=0.9,
                        label=label,
                    )

            except Exception as e:
                print(f"Could not calculate or plot L2 Norm for {algorithm}: {e}")

        for i, comp in enumerate(components):
            ax = ax_flat[i]
            ax.set_ylabel(f"{titles[i]} ({units[i]})")

            ax.grid(True)
            ax.legend()
            ax.set_ylim(bottom=0)

        axs[2, 0].set_xlabel("Time (s)")
        axs[2, 1].set_xlabel("Time (s)")

        # Adjust layout to prevent title overlap
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        print("Generated combined L2 Norm Plot.")

    else:
        print("Skipping L2 norm plot as 'Truth' data is missing.")

    print("Analysis complete. Displaying plots...")
    plt.show()


if __name__ == "__main__":
    main()
