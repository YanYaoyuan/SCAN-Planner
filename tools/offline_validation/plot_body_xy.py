#!/usr/bin/env python3
"""Plot x/y positions from a nav_msgs/Odometry topic in a ROS 2 bag."""

import argparse
import csv
import os
import re

os.environ.setdefault("MPLCONFIGDIR", "/tmp/scanplanner_matplotlib")
os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


DEFAULT_BAG = "/home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08"
DEFAULT_TOPIC = "/body_state_estimation"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", nargs="?", default=DEFAULT_BAG, help="ROS 2 bag directory")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Odometry topic to plot")
    parser.add_argument(
        "-o",
        "--output",
        default="",
        help="Output PNG path; default writes to /tmp",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="Optional CSV output path for index, stamp, x, y, z",
    )
    parser.add_argument("--dpi", type=int, default=160)
    parser.add_argument("--point-size", type=float, default=10.0)
    parser.add_argument("--no-line", action="store_true", help="Draw points only")
    parser.add_argument(
        "--title",
        default="",
        help="Plot title; default is '<bag basename> <topic>'",
    )
    return parser.parse_args()


def safe_name(value):
    value = value.strip("/").replace("/", "_") or "topic"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)


def default_output_path(bag_path, topic):
    bag_name = os.path.basename(os.path.normpath(bag_path))
    return os.path.join("/tmp", f"{bag_name}_{safe_name(topic)}_xy.png")


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def read_xy_points(bag_path, topic):
    reader = open_reader(bag_path)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = "\n".join(f"  {name}: {typ}" for name, typ in sorted(topic_types.items()))
        raise RuntimeError(f"Topic not found in bag: {topic}\nAvailable topics:\n{available}")

    msg_type = get_message(topic_types[topic])
    rows = []
    while reader.has_next():
        name, data, bag_stamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        position = msg.pose.pose.position
        header_stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if header_stamp is not None:
            stamp = float(header_stamp.sec) + float(header_stamp.nanosec) * 1e-9
        else:
            stamp = float(bag_stamp) * 1e-9
        rows.append((len(rows), stamp, float(position.x), float(position.y), float(position.z)))
    return rows, topic_types[topic]


def write_csv(path, rows):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "stamp_sec", "x", "y", "z"])
        writer.writerows(rows)


def plot_xy(path, rows, title, args):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)

    xs = np.array([row[2] for row in rows], dtype=float)
    ys = np.array([row[3] for row in rows], dtype=float)
    indices = np.arange(len(rows), dtype=float)

    fig, ax = plt.subplots(figsize=(8.0, 8.0))
    if not args.no_line:
        ax.plot(xs, ys, color="#4c78a8", linewidth=1.0, alpha=0.65, label="trajectory")
    scatter = ax.scatter(
        xs,
        ys,
        s=args.point_size,
        c=indices,
        cmap="viridis",
        alpha=0.9,
        label="samples",
    )
    ax.scatter([xs[0]], [ys[0]], s=70, color="#2ca02c", marker="o", label="start", zorder=4)
    ax.scatter([xs[-1]], [ys[-1]], s=80, color="#d62728", marker="x", label="end", zorder=4)

    ax.set_title(title)
    ax.set_xlabel("position.x [m]")
    ax.set_ylabel("position.y [m]")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.45)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.colorbar(scatter, ax=ax, label="sample index")
    fig.tight_layout()
    fig.savefig(path, dpi=args.dpi)
    plt.close(fig)


def main():
    args = parse_args()
    if not os.path.isdir(args.bag):
        raise RuntimeError(f"Bag directory does not exist: {args.bag}")

    rows, topic_type = read_xy_points(args.bag, args.topic)
    if not rows:
        raise RuntimeError(f"No messages found on topic: {args.topic}")

    output = args.output or default_output_path(args.bag, args.topic)
    title = args.title or f"{os.path.basename(os.path.normpath(args.bag))} {args.topic}"
    plot_xy(output, rows, title, args)

    if args.csv:
        write_csv(args.csv, rows)

    xs = np.array([row[2] for row in rows], dtype=float)
    ys = np.array([row[3] for row in rows], dtype=float)
    distance = float(np.sum(np.hypot(np.diff(xs), np.diff(ys)))) if len(rows) > 1 else 0.0
    print(f"topic: {args.topic} ({topic_type})")
    print(f"samples: {len(rows)}")
    print(f"x range: [{xs.min():.3f}, {xs.max():.3f}]")
    print(f"y range: [{ys.min():.3f}, {ys.max():.3f}]")
    print(f"xy path length: {distance:.3f} m")
    print(f"plot: {output}")
    if args.csv:
        print(f"csv: {args.csv}")


if __name__ == "__main__":
    main()
