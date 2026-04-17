import rosbag
import bisect

bag_path = 'AMtown03.bag'
output_gt = 'ground_truth.txt'

attitudes = [] # 用于存储 (时间戳, qx, qy, qz, qw)
att_times = [] # 仅存储时间戳用于二分查找

# 1. 提取所有姿态数据
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/dji_osdk_ros/attitude']):
        att_t = msg.header.stamp.to_sec()
        q = msg.quaternion
        attitudes.append((att_t, q.x, q.y, q.z, q.w))
        att_times.append(att_t)

# 2. 提取位置数据，进行时间同步并输出 TUM 格式
with rosbag.Bag(bag_path, 'r') as bag, open(output_gt, 'w') as f:
    for topic, msg, t in bag.read_messages(topics=['/dji_osdk_ros/local_position']):
        pos_t = msg.header.stamp.to_sec()
        
        # 使用二分查找寻找时间戳最接近的姿态
        idx = bisect.bisect_left(att_times, pos_t)
        
        # 边界处理及寻找绝对差值最小的索引
        if idx == 0:
            best_idx = 0
        elif idx == len(att_times):
            best_idx = -1
        else:
            before = att_times[idx - 1]
            after = att_times[idx]
            best_idx = idx if (after - pos_t) < (pos_t - before) else (idx - 1)
            
        best_att = attitudes[best_idx]
        
        # 严格检查时间差 (例如要求偏差小于 0.05 秒)
        if abs(pos_t - best_att[0]) < 0.05:
            # TUM 格式: t tx ty tz qx qy qz qw
            line = f"{pos_t:.6f} {msg.point.x:.6f} {msg.point.y:.6f} {msg.point.z:.6f} {best_att[1]:.6f} {best_att[2]:.6f} {best_att[3]:.6f} {best_att[4]:.6f}\n"
            f.write(line)

print(f"Ground truth saved to {output_gt}")