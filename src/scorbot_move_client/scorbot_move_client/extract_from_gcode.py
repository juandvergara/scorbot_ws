import math
import numpy as np

np.set_printoptions(precision=5, suppress=True)

def extract_values_from_gcode(filename):
    data = []
    last_values = [0, 0, 0, 0, 0, 0, 0]  # Initial values for X, Y, Z, E, R, P, Y, NANOSEC
    time_sum = 0  # Variable to store the cumulative sum of time_nanosec
    sum_time_sec = 0

    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('G1'):
                words = line.split()
                x = next((float(word[1:]) for word in words if word.startswith('X')), last_values[0]) / 1000
                y = next((float(word[1:]) for word in words if word.startswith('Y')), last_values[1]) / 1000
                z = next((float(word[1:]) for word in words if word.startswith('Z')), last_values[2])
                e = next((float(word[1:]) for word in words if word.startswith('E')), last_values[3])
                f = next((float(word[1:]) for word in words if word.startswith('F')), last_values[4])

                if f != last_values[4]:
                    f /= 60 * 1000  # Divide F value by 60
                if z != last_values[2]:
                    z /= 1000

                if last_values == [0, 0, 0, 0, 0, 0, 0]:
                    last_values = [x, y, z, e, f, 180, -90]
                    continue

                distance = math.sqrt((x - last_values[0]) ** 2 + (y - last_values[1]) ** 2 + (z - last_values[2]) ** 2)
                time_sec = int((distance / f))
                time_nanosec = (((distance / f) - time_sec) * 1e9)
                atan2_base = -math.degrees(math.atan2(x, y - 0.170)) - 90

                sum_time_sec += time_sec

                last_values = [x, y, z, e, f, 180, atan2_base]
                row = np.array(last_values[:3] + [0] + last_values[5:] + last_values[3:4] + [sum_time_sec] + [time_nanosec], dtype=np.float64)  # Create the modified row as a NumPy array

                time_sum = time_nanosec  # Update the cumulative sum of time_nanosec
                row[-1] = time_sum  # Assign the cumulative sum to the last element of the row

                data.append(row)  # Append the modified row to data

    return np.array(data)

gcode_filename = '/home/juanmadrid/Escritorio/Shape-Box.gcode'
extracted_data = extract_values_from_gcode(gcode_filename)

print('Extracted Data:')
for row in extracted_data:
    print(row)
