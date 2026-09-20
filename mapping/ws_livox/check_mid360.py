#!/usr/bin/env python3
"""Check the installed MID360 config and UDP bindings before launching."""
import errno
import json
from pathlib import Path
import socket
import sys

from ament_index_python.packages import get_package_share_directory


def main():
    config = Path(get_package_share_directory("livox_ros_driver2")) / "config/MID360_config.json"
    with config.open() as stream:
        settings = json.load(stream)
    host = settings["MID360"]["host_net_info"]
    errors = []
    for channel in ("cmd_data", "push_msg", "point_data", "imu_data", "log_data"):
        address = host[channel + "_ip"]
        if not address:
            continue
        port = host[channel + "_port"]
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((address, port))
        except OSError as error:
            if error.errno == errno.EADDRNOTAVAIL:
                reason = "本机网卡未配置此 IP；请启用雷达有线连接"
            elif error.errno == errno.EADDRINUSE:
                reason = "端口被占用；请检查是否已启动另一个雷达驱动"
            else:
                reason = str(error)
            errors.append(f"{address}:{port}：{reason}")
    if errors:
        print("MID360 启动检查失败：\n" + "\n".join(errors), file=sys.stderr)
        print(f"使用配置：{config}", file=sys.stderr)
        return 1
    print("MID360 本机 IP 和 UDP 端口检查通过（雷达数据仍需实际订阅验证）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
