#!/usr/bin/env python3
# =============================================================================
# 反向工具：把 rscalib 约定的"图序 + video_ts.txt + imu.txt"打成 ROS1 bag，
# 供只吃 rosbag 的外部基线（swift_vio 同步节点 / OpenVINS ros1_serial）离线回放。
# 用 rosbags 库写 bag，**无需安装 ROS**（pip install rosbags）。
#
# 与 ../../rscalib/test/real_equiv/extract_bag.py 互为逆操作，格式约定一致：
#   <in_dir>/cam0/%06d.png     —— 灰度图序（与 video_ts.txt 一一对应）
#   <in_dir>/video_ts.txt      —— 每行一个时间戳（秒）
#   <in_dir>/imu.txt           —— t[s] wx wy wz ax ay az（空格分隔，秒）
# 产出：
#   <out.bag>  话题 /cam0/image_raw (sensor_msgs/Image, mono8)
#              话题 /imu0          (sensor_msgs/Imu)
#
# 用法：
#   python3 frames_to_rosbag.py <in_dir> <out.bag> [--cam-topic /cam0/image_raw] [--imu-topic /imu0]
#   python3 frames_to_rosbag.py --selftest        # 合成自检（不落盘，验证读写往返）
# =============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np


def _typestore():
    from rosbags.typesys import Stores, get_typestore
    return get_typestore(Stores.ROS1_NOETIC)


def _stamp(ts, secs):
    """秒(float) -> builtin_interfaces/Time 兼容对象。"""
    sec = int(secs)
    nanosec = int(round((secs - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return ts.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nanosec)


def _header(ts, secs, frame_id, seq=0):
    return ts.types['std_msgs/msg/Header'](
        seq=seq, stamp=_stamp(ts, secs), frame_id=frame_id)


def _image_msg(ts, secs, img, frame_id='cam0'):
    h, w = img.shape[:2]
    Image = ts.types['sensor_msgs/msg/Image']
    return Image(
        header=_header(ts, secs, frame_id),
        height=h, width=w, encoding='mono8', is_bigendian=0,
        step=w, data=np.ascontiguousarray(img, dtype=np.uint8).reshape(-1))


def _imu_msg(ts, secs, wx, wy, wz, ax, ay, az, frame_id='imu0'):
    Imu = ts.types['sensor_msgs/msg/Imu']
    Vec3 = ts.types['geometry_msgs/msg/Vector3']
    Quat = ts.types['geometry_msgs/msg/Quaternion']
    zeros9 = np.zeros(9, dtype=np.float64)
    return Imu(
        header=_header(ts, secs, frame_id),
        orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
        angular_velocity=Vec3(x=wx, y=wy, z=wz),
        angular_velocity_covariance=zeros9.copy(),
        linear_acceleration=Vec3(x=ax, y=ay, z=az),
        linear_acceleration_covariance=zeros9.copy())


def convert(in_dir, out_bag, cam_topic='/cam0/image_raw', imu_topic='/imu0'):
    import cv2
    from rosbags.rosbag1 import Writer

    in_dir = Path(in_dir)
    ts_file = in_dir / 'video_ts.txt'
    imu_file = in_dir / 'imu.txt'
    cam_dir = in_dir / 'cam0'
    for p in (ts_file, imu_file, cam_dir):
        if not p.exists():
            sys.exit(f'缺少输入: {p}')

    ts_lines = [float(x) for x in ts_file.read_text().split()]
    frames = sorted(cam_dir.glob('*.png'))
    if len(frames) != len(ts_lines):
        sys.exit(f'帧数({len(frames)}) != 时间戳数({len(ts_lines)})')

    imu_rows = []
    for line in imu_file.read_text().splitlines():
        if not line.strip():
            continue
        v = line.split()
        imu_rows.append((float(v[0]),) + tuple(float(x) for x in v[1:7]))

    tsy = _typestore()
    out_bag = Path(out_bag)
    out_bag.parent.mkdir(parents=True, exist_ok=True)
    if out_bag.exists():
        out_bag.unlink()

    with Writer(out_bag) as w:
        cam_conn = w.add_connection(cam_topic, 'sensor_msgs/msg/Image', typestore=tsy)
        imu_conn = w.add_connection(imu_topic, 'sensor_msgs/msg/Imu', typestore=tsy)
        # 按时间戳交错写入（bag 通常要求非严格递增即可；这里各话题独立按序）。
        for path, secs in zip(frames, ts_lines):
            img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            msg = _image_msg(tsy, secs, img)
            w.write(cam_conn, int(secs * 1e9), tsy.serialize_ros1(msg, 'sensor_msgs/msg/Image'))
        for row in imu_rows:
            msg = _imu_msg(tsy, row[0], *row[1:7])
            w.write(imu_conn, int(row[0] * 1e9), tsy.serialize_ros1(msg, 'sensor_msgs/msg/Imu'))

    print(f'写出 {out_bag}: {len(frames)} 帧 -> {cam_topic}, {len(imu_rows)} IMU -> {imu_topic}')


def selftest():
    """合成 3 帧 + 5 条 IMU，序列化再反序列化，验证字段往返一致。"""
    tsy = _typestore()
    img = (np.arange(4 * 6, dtype=np.uint8) % 255).reshape(4, 6)
    im = _image_msg(tsy, 1.5, img)
    raw = tsy.serialize_ros1(im, 'sensor_msgs/msg/Image')
    back = tsy.deserialize_ros1(raw, 'sensor_msgs/msg/Image')
    assert back.height == 4 and back.width == 6 and back.encoding == 'mono8'
    assert np.array_equal(np.asarray(back.data).reshape(4, 6), img)
    assert back.header.stamp.sec == 1 and abs(back.header.stamp.nanosec - 500_000_000) <= 1

    imu = _imu_msg(tsy, 2.25, 0.1, -0.2, 0.3, 0.0, 0.0, 9.81)
    raw = tsy.serialize_ros1(imu, 'sensor_msgs/msg/Imu')
    back = tsy.deserialize_ros1(raw, 'sensor_msgs/msg/Imu')
    assert abs(back.angular_velocity.x - 0.1) < 1e-9
    assert abs(back.linear_acceleration.z - 9.81) < 1e-9
    assert back.header.stamp.sec == 2 and abs(back.header.stamp.nanosec - 250_000_000) <= 1
    print('selftest OK: Image/Imu 序列化往返字段一致')


def main():
    ap = argparse.ArgumentParser(description='帧序列+ts+imu.txt -> ROS1 bag（无需 ROS）')
    ap.add_argument('in_dir', nargs='?', help='含 cam0/、video_ts.txt、imu.txt 的目录')
    ap.add_argument('out_bag', nargs='?', help='输出 .bag 路径')
    ap.add_argument('--cam-topic', default='/cam0/image_raw')
    ap.add_argument('--imu-topic', default='/imu0')
    ap.add_argument('--selftest', action='store_true', help='合成自检，不落盘')
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    if not a.in_dir or not a.out_bag:
        ap.error('需要 <in_dir> <out.bag>，或用 --selftest')
    convert(a.in_dir, a.out_bag, a.cam_topic, a.imu_topic)


if __name__ == '__main__':
    main()
