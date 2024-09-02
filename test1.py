import socket
import struct
import serial
from algs.decentralized import decentralized_algo
from problems.util import *

routes = {
    "Mona1": [(500, 0, 500), (600, 0, 500), (600, 0, 600), (500, 0, 600), (500, 0, 500)],
    "Mona2": [(500, 0, 500), (700, 0, 500), (700, 0, 700), (500, 0, 700), (500, 0, 500)],
}
index_tracker = {}

# Configure format strings
NUM_TRACKERS_FORMAT = 'i'  # An int type, occupies 4 bytes
TRACKER_NAME_FORMAT = '32s'  # 32-byte string
OWL_TRACKER_STATE_FORMAT = (
    '32s i 3f 4f f '  # trackerName[32], ID, position[3], pose[4], headingY
    'I 3f f '  # First marker: ID, position (x, y, z), timestamp
    'I 3f f '  # Second marker
    'I 3f f '  # Third marker
    'I 3f f '  # Fourth marker
)

# Calculate the size of each TrackerState structure
OWL_TRACKER_STATE_SIZE = struct.calcsize(OWL_TRACKER_STATE_FORMAT)  # 148 bytes

Thetas = []


def parse_tracker_state(data):
    """Parse the state data of a single tracker"""
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
    """Parse the entire UDP message"""
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


def get_goal(tracker):
    threshold = 15
    name = tracker['Name'][:5]
    current_location = tracker['Position']

    if name not in routes:
        return None

    route = routes[name]

    if name not in index_tracker:
        index_tracker[name] = 0  # Initialize index to 0

    index = index_tracker[name]

    # Get the current target point
    current_target = route[index]

    # Calculate the difference between the current position and the current target point
    distance = ((current_location[0] - current_target[0]) ** 2 + (current_location[2] - current_target[2]) ** 2) ** 0.5
    print("distance:")
    print(distance)

    # If the difference is less than the threshold, consider it reached
    if distance < threshold:
        index += 1  # Move to the next target point
        if index >= len(route):  # If it's the last target point
            return None
        index_tracker[name] = index  # Update the index in the tracker
        current_target = route[index]  # Get the next target point

    return current_target


def send_tracker_info_to_serial(tracker, serial_port):
    current_goal = get_goal(tracker)

    message = "{}\t{:.1f},{:.1f},{:.1f}\t{:.6f}\t{:.1f},{:.1f},{:.1f}\n".format(
        tracker['Name'][:5],
        tracker['Position'][0], tracker['Position'][1], tracker['Position'][2],
        tracker['HeadingY'],
        current_goal[0], current_goal[1], current_goal[2]
    )
    print(message)
    serial_port.write(message.encode('utf-8'))


def check_and_send_mona_trackers(trackers, serial_port):
    routes_update = get_refs(trackers)
    if routes_update is not None:
        routes.update(routes_update)  # Update the global routes dictionary
        print("Updated routes:", routes)
    for tracker in trackers:
        if (tracker['Name'].startswith('Mona1') or tracker['Name'].startswith('Mona2')) and tracker['Name'] != 'Mona10':
            print(f"Sending Mona tracker data: {tracker['Name']}")
            send_tracker_info_to_serial(tracker, serial_port)


def get_Thetas(trackers):
    Thetas = []
    for tracker in trackers:
        # if (tracker['Name'].startswith('Mona1') or tracker['Name'].startswith('Mona2')) and tracker['Name'] != 'Mona10':
        if (tracker['Name'].startswith('Mona2')) :
            # Get xz position
            xz_position = get_tracker_xz_positions_by_id(trackers, tracker['ID'])
            if xz_position is not None:
                print(f"ID: {tracker['ID']} XZ Position: {xz_position}")
                Thetas.append(xz_position)

    return Thetas


def get_refs(trackers):

    size = 1
    velocity = 1
    k = [0.5, 0.5, 0.5]
    # agents = [Car(size, velocity, k) for _ in range(2)]
    agents = [Car(size, velocity, k)]
    # Obstacles = []
    A_rect = np.array([[-1, 0],
                       [1, 0],
                       [0, -1],
                       [0, 1]])
    b01 = np.array([[-15], [20], [-20], [25]])
    Obstacles = [(A_rect, b01)]
    Thetas = get_Thetas(trackers)

    Goals = []

    pG1 = [50, 50]
    lG1 = [2, 2]
    b1 = np.array([[-(pG1[0] - lG1[0])], [pG1[0] + lG1[0]], [-(pG1[1] - lG1[1])], [pG1[1] + lG1[1]]])
    Goal1 = (A_rect, b1)
    Goals.append(Goal1)

    # pG2 = [70, 70]
    # lG2 = [2, 2]
    # b2 = np.array([[-(pG2[0] - lG2[0])], [pG2[0] + lG2[0]], [-(pG2[1] - lG2[1])], [pG2[1] + lG2[1]]])
    # Goal2 = (A_rect, b2)
    # Goals.append(Goal2)

    limits = [[-100, 100], [-100, 100]]
    min_segs, max_segs, obs_steps = read_configuration("problems/zigzag/config.yaml")

    # Check if Thetas has been populated
    if len(Thetas) == 0:
        print("Error: Thetas is empty. Please ensure it is initialized correctly.")
        return None

    routes_update = {}  # Create a dictionary to store the updated routes

    try:
        refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
        for idx in range(len(refs)):
            route = [(p[1]*100, 0, p[2]*100) for p in refs[idx]]
            tracker_name = f'Mona{idx + 2}'  # Assume names like 'Mona1', 'Mona2', etc.
            routes_update[tracker_name] = route  # Store the updated routes in the dictionary
            # print(f"Route for {tracker_name}: {route}")

        if refs is None:
            print("Error: decentralized_algo returned None. Please check input parameters and function logic.")
            return None
    except IndexError as e:
        print(f"IndexError caught: {e}")
        return None

    return routes_update

def get_tracker_xz_positions_by_id(trackers, tracker_id):
    """
    Get the xz position of a specific tracker ID, convert to integers, and scale down
    :param trackers: List of tracker data
    :param tracker_id: Target tracker's ID
    :return: Target tracker's xz position [x, z]
    """
    for tracker in trackers:
        if tracker['ID'] == tracker_id:
            # Check the validity of tracker['Position']
            x = int(tracker['Position'][0] / 100)
            z = int(tracker['Position'][2] / 100)

            # Debug output
            # print(f"Tracker ID: {tracker_id}, Raw Position: {tracker['Position']}, Computed XZ: [{x}, {z}]")

            return [x, z]

    return None


def main():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 7777))

    # Open the serial port
    serial_port = serial.Serial('COM6', 115200, timeout=1)  # Adjust the serial port and baud rate as needed

    while True:
        data, addr = sock.recvfrom(2048)
        trackers = parse_udp_message(data)
        # get_Thetas(trackers)
        # print(f"Thetas: {Thetas}")
        # get_refs(trackers)

        check_and_send_mona_trackers(trackers, serial_port)

if __name__ == "__main__":
    main()
