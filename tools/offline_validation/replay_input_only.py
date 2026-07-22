#!/usr/bin/env python3
"""Replay selected input topics from a ROS 2 sqlite3 bag.

This avoids ros2 bag play --topics corner cases where a player node exists but
does not create publishers for the requested input topics.
"""

import argparse
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from std_msgs.msg import Bool


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/state_estimation", "/cloud_registered"],
        help="Topics to publish from the bag",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="Playback rate")
    parser.add_argument("--loop", action="store_true", help="Loop when the bag ends")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many wall-clock seconds; 0 means no limit",
    )
    parser.add_argument(
        "--qos",
        choices=["best_effort", "reliable"],
        default="best_effort",
        help="Publisher reliability",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=100,
        help="Print counters every N messages per topic",
    )
    parser.add_argument(
        "--pause-topic",
        default="/scanplanner/input_replay_pause",
        help="std_msgs/Bool topic used to pause/resume playback",
    )
    parser.add_argument(
        "--start-paused",
        action="store_true",
        help="Start playback paused; resume via the pause topic",
    )
    return parser.parse_args()


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def main():
    args = parse_args()
    wanted_topics = set(args.topics)
    reliability = (
        ReliabilityPolicy.RELIABLE
        if args.qos == "reliable"
        else ReliabilityPolicy.BEST_EFFORT
    )
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = rclpy.create_node("scanplanner_input_only_replay")
    reader = open_reader(args.bag)
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    missing = sorted(topic for topic in wanted_topics if topic not in type_map)
    if missing:
        raise RuntimeError(f"bag does not contain requested topics: {missing}")

    msg_types = {topic: get_message(type_map[topic]) for topic in wanted_topics}
    pubs = {topic: node.create_publisher(msg_types[topic], topic, qos) for topic in wanted_topics}

    first_bag_time = None
    state = {
        "first_wall_time": None,
        "paused": args.start_paused,
        "paused_since": time.monotonic() if args.start_paused else None,
        "duration_start_wall_time": time.monotonic(),
    }
    count = {topic: 0 for topic in wanted_topics}
    loops = 0

    def duration_expired():
        return (
            args.duration > 0.0
            and time.monotonic() - state["duration_start_wall_time"] >= args.duration
        )

    def pause_callback(msg):
        requested_pause = bool(msg.data)
        if requested_pause == state["paused"]:
            return

        now = time.monotonic()
        state["paused"] = requested_pause
        if requested_pause:
            state["paused_since"] = now
            print("Input replay paused", flush=True)
            return

        paused_since = state["paused_since"]
        if paused_since is not None:
            state["duration_start_wall_time"] += now - paused_since
        if paused_since is not None and state["first_wall_time"] is not None:
            paused_since = max(paused_since, state["first_wall_time"])
            state["first_wall_time"] += now - paused_since
        state["paused_since"] = None
        print("Input replay resumed", flush=True)

    node.create_subscription(Bool, args.pause_topic, pause_callback, QoSProfile(depth=1))

    def wait_while_paused():
        while rclpy.ok() and state["paused"]:
            rclpy.spin_once(node, timeout_sec=0.1)
        return rclpy.ok()

    def wait_for_bag_stamp(stamp):
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not wait_while_paused():
                return False

            target_wall_time = (
                state["first_wall_time"] + (stamp - first_bag_time) * 1e-9 / args.rate
            )
            delay = target_wall_time - time.monotonic()
            if delay <= 0:
                return True
            rclpy.spin_once(node, timeout_sec=min(delay, 0.05))
        return False

    print(
        f"Replaying {sorted(wanted_topics)} from {args.bag} "
        f"at rate {args.rate:g}, qos={args.qos}, loop={args.loop}, "
        f"pause_topic={args.pause_topic}, start_paused={args.start_paused}",
        flush=True,
    )

    try:
        while rclpy.ok():
            if duration_expired():
                break

            if not wait_while_paused():
                break

            if not reader.has_next():
                if not args.loop:
                    break
                loops += 1
                print(f"Loop {loops} complete, counts={count}", flush=True)
                reader = open_reader(args.bag)
                first_bag_time = None
                state["first_wall_time"] = None
                continue

            topic, data, stamp = reader.read_next()
            if topic not in wanted_topics:
                continue

            if first_bag_time is None:
                first_bag_time = stamp
                state["first_wall_time"] = time.monotonic()

            if not wait_for_bag_stamp(stamp):
                break

            if not rclpy.ok():
                break
            msg = deserialize_message(data, msg_types[topic])
            try:
                pubs[topic].publish(msg)
            except Exception:
                if rclpy.ok():
                    raise
                break
            count[topic] += 1
            if count[topic] == 1 or (
                args.print_every > 0 and count[topic] % args.print_every == 0
            ):
                stamp_msg = getattr(getattr(msg, "header", None), "stamp", "")
                print(f"published {topic}: {count[topic]} stamp={stamp_msg}", flush=True)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Replay stopped, counts={count}", flush=True)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
