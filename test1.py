import socket
import struct
import serial
from algs.decentralized import decentralized_algo
from problems.util import *


# 配置格式化字符串
NUM_TRACKERS_FORMAT = 'i'  # 一个int类型，占4个字节
TRACKER_NAME_FORMAT = '32s'  # 32字节字符串
OWL_TRACKER_STATE_FORMAT = (
    '32s i 3f 4f f '  # trackerName[32], ID, position[3], pose[4], headingY
    'I 3f f '  # First marker: ID, position (x, y, z), timestamp
    'I 3f f '  # Second marker
    'I 3f f '  # Third marker
    'I 3f f '  # Fourth marker
)

# 计算每个TrackerState结构体的大小
OWL_TRACKER_STATE_SIZE = struct.calcsize(OWL_TRACKER_STATE_FORMAT)  # 148字节

Thetas = []

index_tracker = {}
def parse_tracker_state(data):
    """解析单个tracker的状态数据"""
    unpacked_data = struct.unpack(OWL_TRACKER_STATE_FORMAT, data)
    name = unpacked_data[0].decode('latin1').strip('\x00')
    tracker_id = unpacked_data[1]
    position = unpacked_data[2:5]
    pose = unpacked_data[5:9]
    headingY = unpacked_data[9]

    markers = []
    marker_start = 10
    for i in range(4):
        marker_id = unpacked_data[marker_start]
        marker_position = unpacked_data[marker_start + 1:marker_start + 4]
        marker_timestamp = unpacked_data[marker_start + 4]
        markers.append({
            'ID': marker_id,
            'Position': marker_position,
            'Timestamp': marker_timestamp
        })
        marker_start += 5

    return {
        'Name': name,
        'ID': tracker_id,
        'Position': position,
        'Pose': pose,
        'HeadingY': headingY,
        'Markers': markers
    }

def parse_udp_message(data):
    """解析整个UDP消息"""
    num_trackers = struct.unpack(NUM_TRACKERS_FORMAT, data[:4])[0]
    # print(f"num_trackers:{num_trackers}")
    tracker_states = []

    offset = 4
    for _ in range(num_trackers):
        tracker_name_data = data[offset:offset + 32]
        offset +=32
        tracker_state_data = data[offset:offset + OWL_TRACKER_STATE_SIZE]

        if len(tracker_state_data) != OWL_TRACKER_STATE_SIZE:
            print(
                f"Error: Data length {len(tracker_state_data)} does not match expected size {OWL_TRACKER_STATE_SIZE}.")
            offset += OWL_TRACKER_STATE_SIZE
            continue

        tracker_info = parse_tracker_state(tracker_state_data)
        tracker_states.append(tracker_info)

        offset += OWL_TRACKER_STATE_SIZE

    return tracker_states



def get_goal(tracker, refs):
    """
    根据tracker的当前位置和refs中的路径计算下一个目标点
    :param tracker: 追踪器字典，包含位置和名称等信息
    :param refs: 包含所有路径的全局列表
    :return: 当前目标点的坐标 (x, 0, y)
    """
    threshold = 15  # 到达目标点的阈值距离
    name = tracker['Name'][:5]  # 提取追踪器的名称前5个字符
    current_location = tracker['Position']  # 获取追踪器的当前位置坐标 (x, y, z)

    # 根据名称找到对应的 refs 索引
    if name not in index_tracker:
        index_tracker[name] = 0  # 初始化该追踪器的路径索引为0（开始点）

    idx = int(name[-1]) - 1  # 假设'Mona1' -> 0, 'Mona2' -> 1，依此类推
    if idx >= len(refs):
        return None  # 如果索引超出范围，返回None

    route = [(p[1]*10, 0, p[2]*10) for p in refs[idx]]  # 转换refs为(x, 0, y)格式的路径

    index = index_tracker[name]  # 获取当前的路径索引

    # 获取当前的目标点
    current_target = route[index]

    # 计算当前位置与当前目标点之间的欧几里得距离
    distance = ((current_location[0] - current_target[0]) ** 2 + (current_location[2] - current_target[2]) ** 2) ** 0.5
    # print("distance:")
    # print(distance)

    # 如果距离小于阈值，认为已经到达当前目标点
    if distance < threshold:
        index += 1  # 更新索引到下一个目标点
        if index >= len(route):  # 如果已经是最后一个目标点
            return None  # 返回None，表示没有更多目标点
        index_tracker[name] = index  # 更新追踪器的路径索引
        current_target = route[index]  # 获取下一个目标点

    return current_target  # 返回当前的目标点坐标 (x, 0, y)


def get_tracker_xz_positions_by_id(trackers, tracker_id):
    """
    获取特定ID的小车的xz位置，转换为整数并缩小比例
    :param trackers: 追踪器数据列表
    :param tracker_id: 目标小车的ID
    :return: 目标小车的xz位置 [x, z]
    """
    for tracker in trackers:
        if tracker['ID'] == tracker_id:
            # 检查 tracker['Position'] 的有效性
            x = int(tracker['Position'][0] / 100)
            z = int(tracker['Position'][2] / 100)

            # 调试输出
            print(f"Tracker ID: {tracker_id}, Raw Position: {tracker['Position']}, Computed XZ: [{x}, {z}]")

            return [x, z]

    # 如果没有匹配到ID，返回None而不是[0, 0]
    return None

def send_tracker_info_to_serial(tracker, serial_port):
    """发送解析后的tracker信息到串行端口"""
    message = "{}\t{:.1f},{:.1f},{:.1f}\t{:.6f}\n".format(
        tracker['Name'][:5],
        tracker['Position'][0], tracker['Position'][1], tracker['Position'][2],
        tracker['HeadingY']
    )
    print(message)
    serial_port.write(message.encode('utf-8'))


# def send_tracker_info_to_serial(tracker, serial_port, refs):
#     """发送解析后的tracker信息到串行端口"""
#     current_goal = get_goal(tracker, refs)
#
#     if current_goal is None:
#         return  # 如果没有目标点，直接返回
#
#     message = "{}\t{:.1f},{:.1f},{:.1f}\t{:.6f}\t{:.1f},{:.1f},{:.1f}\n".format(
#         tracker['Name'][:5],
#         tracker['Position'][0], tracker['Position'][1], tracker['Position'][2],
#         tracker['HeadingY'],
#         current_goal[0], current_goal[1], current_goal[2]
#     )
#     # print(message)
#     serial_port.write(message.encode('utf-8'))


# def check_and_send_mona_trackers(trackers, serial_port, refs):
#     """检查tracker的名称是否以'Mona1'或'Mona2'开头，并将数据发送到串行端口，同时打印xz位置"""
#     global Thetas  # 使用全局变量 Thetas
#     Thetas = []  # 每次调用都重新初始化 Thetas 列表
#     for tracker in trackers:
#         if (tracker['Name'].startswith('Mona1') or tracker['Name'].startswith('Mona2')) and tracker['Name'] != 'Mona10':
#             # 获取 xz 位置
#             xz_position = get_tracker_xz_positions_by_id(trackers, tracker['ID'])
#
#             if xz_position is not None:
#                 print(f"ID: {tracker['ID']} XZ Position: {xz_position}")
#                 Thetas.append(xz_position)
#
#             # print(f"Sending Mona tracker data: {tracker['Name']}")
#             send_tracker_info_to_serial(tracker, serial_port, refs)
#
#     return Thetas

def check_and_send_mona_trackers(trackers, serial_port):
    """检查tracker的名称是否以'Mona1'或'Mona2'开头，并将数据发送到串行端口，同时打印xz位置"""
    global Thetas  # 使用全局变量 Thetas
    Thetas = []  # 每次调用都重新初始化 Thetas 列表
    for tracker in trackers:
        if (tracker['Name'].startswith('Mona1') or tracker['Name'].startswith('Mona2')) and tracker['Name'] != 'Mona10':
            # 获取 xz 位置
            xz_position = get_tracker_xz_positions_by_id(trackers, tracker['ID'])

            if xz_position is not None:
                print(f"ID: {tracker['ID']} XZ Position: {xz_position}")
                Thetas.append(xz_position)

            # print(f"Sending Mona tracker data: {tracker['Name']}")
            send_tracker_info_to_serial(tracker, serial_port)

    return Thetas
def get_refs(Thetas):

    size = 1
    velocity = 1
    k = [0.5, 0.5, 0.5]
    agents = [Car(size, velocity, k) for _ in range(2)]
    Obstacles = []
    A_rect = np.array([[-1, 0],
                       [1, 0],
                       [0, -1],
                       [0, 1]])

    Goals = []

    pG1 = [50, 50]
    lG1 = [2, 2]
    b1 = np.array([[-(pG1[0] - lG1[0])], [pG1[0] + lG1[0]], [-(pG1[1] - lG1[1])], [pG1[1] + lG1[1]]])
    Goal1 = (A_rect, b1)
    Goals.append(Goal1)

    pG2 = [70, 70]
    lG2 = [2, 2]
    b2 = np.array([[-(pG2[0] - lG2[0])], [pG2[0] + lG2[0]], [-(pG2[1] - lG2[1])], [pG2[1] + lG2[1]]])
    Goal2 = (A_rect, b2)
    Goals.append(Goal2)

    limits = [[-100, 100], [-100, 100]]
    min_segs, max_segs, obs_steps = read_configuration("problems/zigzag/config.yaml")


    # 检查 Thetas 是否已经被填充
    if len(Thetas) == 0:
        print("Error: Thetas is empty. Please ensure it is initialized correctly.")
        return None

    try:
        refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
        for idx in range(len(refs)):
            # 生成路径点，以 (x, 0, y) 的格式
            route = [(p[1] , 0, p[2]) for p in refs[idx]]

            # 打印路径
            print(f"Route for idx {idx}: {route}")
        if refs is None:
            print("Error: decentralized_algo returned None. Please check input parameters and function logic.")
            return None
    except IndexError as e:
        print(f"IndexError caught: {e}")
        return None

    return refs

def send_refs_to_serial(refs, serial_port):
    """发送 refs 路径点到串行端口"""
    for idx, route in enumerate(refs):
        for p in route:
            x, z = p[1], p[2]  # 取出 x 和 z 坐标
            message = f"Route Point (idx {idx}): {x:.1f},0.0,{z:.1f}\n"
            print(message)  # 打印到控制台，用于调试
            serial_port.write(message.encode('utf-8'))  # 发送到串口




def main():
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 7777))

    # 打开串行端口
    serial_port = serial.Serial('COM6', 115200, timeout=1)  # 根据实际情况调整串行端口和波特率

    while True:
        data, addr = sock.recvfrom(2048)
        trackers = parse_udp_message(data)

        check_and_send_mona_trackers(trackers, serial_port)
        print(f"Thetas: {Thetas}")
        # refs = get_refs(Thetas)
        # if refs is not None:
        #     send_refs_to_serial(refs, serial_port)

if __name__ == "__main__":
    main()
