#!/usr/bin/env python3

import yaml
import math
import numpy as np


def slerp(q1, q2, t):
    # Convert quaternion dictionary to numpy array
    #print(f"Test q1: {q1['w']}, {q1['x']}, {q1['y']}, {q1['z']}")
    #print(f"Test q2: {q2['w']}, {q2['x']}, {q2['y']}, {q2['z']}")
    #print('+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-')
    q1 = np.array([q1['w'], q1['x'], q1['y'], q1['z']])
    q2 = np.array([q2['w'], q2['x'], q2['y'], q2['z']])

    if np.allclose(q1, q2) or np.allclose(q1, -q2):
        return {
            'w': float(q1[0]),
            'x': float(q1[1]),
            'y': float(q1[2]),
            'z': float(q1[3])
        }

    # Normalize quaternions
    q1 /= np.linalg.norm(q1)
    q2 /= np.linalg.norm(q2)

    # Dot product to get the angle between quaternions
    dot_product = np.dot(q1, q2)
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Calculate the angle and axis of rotation
    theta = np.arccos(dot_product)
    axis_of_rotation = np.cross(q1[1:], q2[1:])
    axis_of_rotation /= np.linalg.norm(axis_of_rotation)

    # Interpolate
    q_interpolated = np.sin((1 - t) * theta) / np.sin(theta) * q1 + np.sin(t * theta) / np.sin(theta) * q2

    # Normalize the result and return as a dictionary
    q_interpolated /= np.linalg.norm(q_interpolated)
    return {
        'w': float(q_interpolated[0]),
        'x': float(q_interpolated[1]),
        'y': float(q_interpolated[2]),
        'z': float(q_interpolated[3])
    }



def point_distance(p1, p2):
    return math.sqrt(
                    (p1['x'] - p2['x']) * (p1['x'] - p2['x']) +
                    (p1['y'] - p2['y']) * (p1['y'] - p2['y']) +
                    (p1['z'] - p2['z']) * (p1['z'] - p2['z'])
    )


def intermediate_position(p1, p2, t):
    return {
        'x': (1 - t) * p1['x'] + t * p2['x'],
        'y': (1 - t) * p1['y'] + t * p2['y'],
        'z': (1 - t) * p1['z'] + t * p2['z']
    }


def orientation_to_rotation_matrix(orientation):
    w, x, y, z = orientation.values()
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return rotation_matrix


def side_boxes(center_position, center_orientation, box_size, separation):
    rotation_matrix = orientation_to_rotation_matrix(center_orientation)
    center_position = np.array(list(center_position.values()))

    # Calculate the offset for the side boxes
    offset_y_positive = np.array([0, 1, 0]) * (box_size / 2 + separation)
    offset_y_negative = -offset_y_positive

    # Calculate the positions of the side boxes
    side_position_y_positive = center_position + np.dot(rotation_matrix, offset_y_positive)
    side_position_y_negative = center_position + np.dot(rotation_matrix, offset_y_negative)

    side_position_y_positive = {
        'x': float(side_position_y_positive[0]),
        'y': float(side_position_y_positive[1]),
        'z': float(side_position_y_positive[2])
    }

    side_position_y_negative = {
        'x': float(side_position_y_negative[0]),
        'y': float(side_position_y_negative[1]),
        'z': float(side_position_y_negative[2])
    }

    return side_position_y_positive, side_position_y_negative




def create_box(box_id: int, box_size: float,
               position, orientation, box_type: str):
    print(f"TEST: new_box #{box_id} position {position}")
    #print(f"TEST: {segment_box['orientation']}")
    segment_box = {}
    segment_box['position'] = {
        'x': position['x'],
        'y': position['y'],
        'z': position['z']
    }
    segment_box['orientation'] = {
        'w': orientation['w'],
        'x': orientation['x'],
        'y': orientation['y'],
        'z': orientation['z']
    }
    segment_box['dimensions'] = {
        'x': box_size,
        'y': box_size,
        'z': box_size
    }
    segment_box['accumulate_alarm_size'] = 1
    segment_box['accumulate_reference'] = 60
    segment_box['accumulate_duration'] = 2.0
    segment_box['dio'] = -1 # digital output
    segment_box['dio_time_out'] = 1.0
    segment_box['event_hysteresis_seconds'] = 0.0
    segment_box['event_time_out_seconds'] = 1.5
    segment_box['excluded_classes'] = []
    segment_box['hysteresis_threshold'] = 3
    segment_box['hysteresis_window_size'] = 6
    segment_box['id'] = box_id
    segment_box['logic_id_1'] = 0
    segment_box['logic_id_2'] = 0
    segment_box['logic_op'] = ''
    segment_box['name'] = ''
    segment_box['resolution'] = 0.01
    segment_box['save_alarm_box_diff'] = 0
    segment_box['save_alarm_scan'] = 0
    segment_box['save_duration'] = 100
    segment_box['save_interval'] = 0
    segment_box['standard_deviation_hysteresis_threshold'] = 3
    segment_box['standard_deviation_hysteresis_window_size'] = 6
    segment_box['standard_deviation_save'] = 0
    segment_box['standard_deviation_threshold'] = 0.0
    segment_box['threshold'] = 20
    segment_box['threshold_percent'] = 0.0
    segment_box['type'] = box_type
    return segment_box


# Read keyframes from file
box_coords = {}
with open("input_boxes.yaml", "r") as read_file:
    try:
        yaml_read = yaml.safe_load(read_file)

        for object in yaml_read:
            if object.isdigit():
                #print(f"\nBox: {object} {yaml_read[object]['dimensions']}")
                box_coords[object] = {}
                box_coords[object]['dimensions'] = yaml_read[object]['dimensions']
                box_coords[object]['orientation'] = yaml_read[object]['orientation']
                box_coords[object]['position'] = yaml_read[object]['position']
    except yaml.YAMLError as err:
        print(err)


# Order and obtain list of keyframes
key_boxes = [box_coords[key] for key in \
        sorted(box_coords.keys(), reverse=False)]

print("-------------------")
print(key_boxes)


# Create final boxes from keyframes
yaml_write = {}
circuit_box_counter = 0
border_box_counter  = 0
box_size = .5
separation = .8
circuit_start = 1000
border_start = 2000
for idx, box in enumerate(key_boxes):
    print(f'\nCreating YAML. Box {idx + 1} of {len(key_boxes)}')

    distance = point_distance(key_boxes[idx]['position'], key_boxes[(idx + 1) % len(key_boxes)]['position'])
    n_segments = math.ceil(distance / box_size)
    print(f"N of segments: {n_segments} = ceil(distance {distance} / box_size {box_size})")

    for segment in range(n_segments):
        segment_position = intermediate_position(
            key_boxes[idx]['position'],
            key_boxes[(idx + 1) % len(key_boxes)]['position'],
            segment / n_segments
        )

        segment_orientation = slerp(
            key_boxes[idx]['orientation'],
            key_boxes[(idx + 1) % len(key_boxes)]['orientation'],
            segment / n_segments
        )
        
        circuit_box = create_box(
            circuit_box_counter + circuit_start,
            box_size,
            segment_position,
            segment_orientation,
            'intrusion detection'
        )
        yaml_write[str(circuit_box_counter + circuit_start)] = circuit_box
        circuit_box_counter += 1

        for side_position in side_boxes(segment_position, segment_orientation, box_size, separation):
            print(f"Side box position: {side_position}")
            border_box = create_box(
                border_box_counter + border_start,
                box_size,
                side_position,
                segment_orientation,
                'tracking'
            )
            yaml_write[str(border_box_counter + border_start)] = border_box
            border_box_counter += 1

yaml_write['box_v'] = list(yaml_write.keys())
yaml_write['last_id'] = border_box_counter + border_start - 1


# Write result to file
with open("output_boxes.yaml", "w") as write_file:
    #yaml.dump(yaml_write, write_file, default_style='"', default_flow_style=False)
    yaml.dump(yaml_write, write_file)


print("\nEnd")
