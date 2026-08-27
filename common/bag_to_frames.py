#!/usr/bin/env python3
# =============================================================================
# 把 ROS1 bag 拆成 karpenko/rscalib 约定的"图序 + video_ts.txt + imu.txt"。
# 是 frames_to_rosbag.py 的逆操作；用 rosbags 库读 bag，**无需安装 ROS**。
#
# 产出（<out_dir>/）：
#   cam0/%06d.png          —— 8-bit 灰度图序（与 video_ts.txt 一一对应）
#   video_ts.txt           —— 每行一个相机帧时间戳（秒，取 header.stamp）
#   imu.txt                —— t wx wy wz ax ay az（空格分隔，秒 / rad·s⁻¹ / m·s⁻²）
#
# 单目相机话题用 --cam-topic 指定（TUM-RSVI：/cam1/image_raw=RS，/cam0/image_raw=GS）。
# mono16 图像按 >>8 降为 mono8（TUM-RSVI 为满 16-bit 量程，右移 8 位保留对比度）。
#
# 用法：
#   python3 bag_to_frames.py <in.bag> <out_dir> [--cam-topic /cam1/image_raw] [--imu-topic /imu0]
# =============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np


def _typestore():
    from rosbags.typesys import Stores, get_typestore
    return get_typestore(Stores.ROS1_NOETIC)


def _stamp_secs(hdr):
    return hdr.stamp.sec + hdr.stamp.nanosec * 1e-9


def _to_mono8(msg):
    enc = msg.encoding
    if enc == 'mono8':
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    elif enc == 'mono16':
        a = np.frombuffer(msg.data, dtype='<u2').reshape(msg.height, msg.width)
        a = (a >> 8).astype(np.uint8)   # 满 16-bit 量程 → 高字节
    elif enc in ('bgr8', 'rgb8'):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        a = a[..., ::-1] if enc == 'rgb8' else a
        a = (0.299 * a[..., 2] + 0.587 * a[..., 1] + 0.114 * a[..., 0]).astype(np.uint8)
    else:
        sys.exit(f'不支持的图像编码: {enc}')
    return a


def convert(in_bag, out_dir, cam_topic='/cam1/image_raw', imu_topic='/imu0'):
    import cv2
    from rosbags.rosbag1 import Reader

    in_bag = Path(in_bag)
    out_dir = Path(out_dir)
    cam_dir = out_dir / 'cam0'          # 沿用 karpenko/rscalib 约定的 cam0/ 子目录名
    cam_dir.mkdir(parents=True, exist_ok=True)
    tsy = _typestore()

    ts_lines, imu_rows = [], []
    with Reader(in_bag) as r:
        topics = {c.topic for c in r.connections}
        for t in (cam_topic, imu_topic):
            if t not in topics:
                sys.exit(f'bag 中无话题 {t}；可用: {sorted(topics)}')
        cam_conn = [c for c in r.connections if c.topic == cam_topic]
        imu_conn = [c for c in r.connections if c.topic == imu_topic]

        n = 0
        for conn, _, raw in r.messages(connections=cam_conn):
            m = tsy.deserialize_ros1(raw, conn.msgtype)
            cv2.imwrite(str(cam_dir / f'{n:06d}.png'), _to_mono8(m))
            ts_lines.append(_stamp_secs(m.header))
            n += 1
        for conn, _, raw in r.messages(connections=imu_conn):
            m = tsy.deserialize_ros1(raw, conn.msgtype)
            imu_rows.append((_stamp_secs(m.header),
                             m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                             m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

    (out_dir / 'video_ts.txt').write_text('\n'.join(f'{t:.9f}' for t in ts_lines) + '\n')
    with open(out_dir / 'imu.txt', 'w') as f:
        f.write('# t wx wy wz ax ay az\n')
        for row in imu_rows:
            f.write(' '.join(f'{v:.9f}' for v in row) + '\n')
    print(f'写出 {out_dir}: {len(ts_lines)} 帧 ({cam_topic}) + {len(imu_rows)} IMU ({imu_topic})')


def main():
    ap = argparse.ArgumentParser(description='ROS1 bag -> 帧序列+video_ts.txt+imu.txt（无需 ROS）')
    ap.add_argument('in_bag')
    ap.add_argument('out_dir')
    ap.add_argument('--cam-topic', default='/cam1/image_raw')
    ap.add_argument('--imu-topic', default='/imu0')
    a = ap.parse_args()
    convert(a.in_bag, a.out_dir, a.cam_topic, a.imu_topic)


if __name__ == '__main__':
    main()
