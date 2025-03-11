#!/bin/bash

# 输出开始检查的信息
echo "Checking for video devices..."

# 查询当前可用的video设备
video_devices=$(ls /dev/ |grep depth_cam 2>/dev/null)

echo $video_devices

# 如果查询结果为空，则重启所有USB设备
if [ -z "$video_devices" ]; then
  echo "No video devices detected. Restarting all USB devices..."

  for i in /sys/bus/usb/devices/*/ ; do
    if [ -e "$i"authorized ]; then
      echo 0 > "$i"authorized 
      echo 1 > "$i"authorized
    fi
  done

  # 重启USB设备后，持续检测video设备
  while : ; do
    # 再次查询video设备
    video_devices=$(ls /dev/ |grep depth_cam 2>/dev/null)
    
    # 如果video设备不为空，跳出循环
    if [ -n "$video_devices" ]; then
      echo "Video device detected: $video_devices"
      break
    fi

    # 如果video设备仍为空，等待一段时间后再次检测
    sleep 1
  done
else
  echo "Video device detected: $video_devices"
fi
sleep 2
