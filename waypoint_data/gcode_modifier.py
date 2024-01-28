def scale_offset_gcode(input_file, output_file, scale_factor, x_offset, y_offset):
    with open(input_file, 'r') as infile:
        lines = infile.readlines()

    first_x = None
    first_y = None

    modified_lines = []
    for line in lines:
        if line.startswith(('N', 'G0', 'G1')):
            words = line.split()
            modified_words = []
            for word in words:
                if word.startswith(('X', 'Y')):
                    axis, value = word[0], float(word[1:])
                    if axis == 'X' and first_x is None:
                        first_x = value
                    elif axis == 'Y' and first_y is None:
                        first_y = value

                    if axis == 'X':
                        modified_value = (value - first_x) * scale_factor + x_offset
                    else:
                        modified_value = (value - first_y) * scale_factor + y_offset
                    modified_words.append(f"{axis}{modified_value:.6f}")
                else:
                    modified_words.append(word)
            modified_line = ' '.join(modified_words)
            modified_lines.append(modified_line + '\n')
        else:
            modified_lines.append(line)

    with open(output_file, 'w') as outfile:
        outfile.writelines(modified_lines)

if __name__ == "__main__":
    input_file = "src/Light-Painting-Robot/waypoint_data/input/Northwestern_Wildcats_logo.gcode"  # Replace with your input file name
    output_file = "src/Light-Painting-Robot/waypoint_data/output/modified_Northwestern_Wildcats_logo.gcode"  # Replace with your desired output file name
    scale_factor = 1.0  # Replace with your desired scale factor
    x_offset = 0.0  # Replace with your desired X offset in mm
    y_offset = 0.0   # Replace with your desired Y offset in mm

    scale_offset_gcode(input_file, output_file, scale_factor, x_offset, y_offset)