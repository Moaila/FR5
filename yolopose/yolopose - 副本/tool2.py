import os


def remove_lines_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            if not line.strip().startswith('3'):
                file.write(line)


def process_directory(directory):
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):
            file_path = os.path.join(directory, filename)
            remove_lines_from_file(file_path)


process_directory('D:\\Project\\Dian\\yolopose\\temp')
