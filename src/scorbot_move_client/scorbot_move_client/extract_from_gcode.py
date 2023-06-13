import math
import numpy as np

np.set_printoptions(precision=5, suppress=True)


def extract_values_from_gcode(filename):
    data = []
    # Initial values for X, Y, Z, E, R, P, Y, NANOSEC
    last_values = [0, 0, 0, 0, 0, 0, 0]
    time_sum_sec = 0  # Variable to store the cumulative sum of time_nanosec

    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('G1'):
                words = line.split()
                x = next((float(word[1:]) for word in words if word.startswith(
                    'X')), last_values[0]) / 1000
                y = next((float(word[1:]) for word in words if word.startswith(
                    'Y')), last_values[1]) / 1000
                z = next(
                    (float(word[1:]) for word in words if word.startswith('Z')), last_values[2])
                e = next(
                    (float(word[1:]) for word in words if word.startswith('E')), last_values[3])
                f = next(
                    (float(word[1:]) for word in words if word.startswith('F')), last_values[4])

                if f != last_values[4]:
                    f /= 60 * 1000  # Divide F value by 60
                if z != last_values[2]:
                    z /= 1000

                if last_values == [0, 0, 0, 0, 0, 0, 0]:
                    last_values = [x, y, z, e, f, 180, -90]
                    continue

                distance = math.sqrt(
                    (x - last_values[0]) ** 2 + (y - last_values[1]) ** 2 + (z - last_values[2]) ** 2)
                time_sum_sec += (distance / f)
                time_sec = int(time_sum_sec)
                time_nano_sec = int((time_sum_sec - time_sec) * 1e9)
                atan2_base = -math.degrees(math.atan2(x, y - 0.170)) - 90

                last_values = [x, y, z, e, f, 180, atan2_base]
                row = np.array(last_values[:3] + [0] + last_values[5:] + last_values[3:4] + [time_sec] + [
                               time_nano_sec], dtype=np.float64)  # Create the modified row as a NumPy array

                data.append(row)  # Append the modified row to data

    return np.array(data)


gcode_filename = '/home/juanmadrid/Escritorio/Shape-Box.gcode'
extracted_data = extract_values_from_gcode(gcode_filename)

print('Extracted Data:')
for row in extracted_data:
    print(row)
