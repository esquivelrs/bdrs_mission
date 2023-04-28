import glob
import numpy as np
from matplotlib import pyplot as plt


def get_logs(logs_file_path: str):
    logs = sorted(glob.glob(logs_file_path))
    return logs


def get_log_contents(log_file_path: str) -> str:
    with open(log_file_path) as f:
        contents = f.read()
    return contents


def find_data_start_index(data_array: list[list[str]]):
    data_index = None
    for index, line in enumerate(data_array):
       if not '%' in line[0]:
            data_index = index
            return data_index
    return data_index


def parse_content(content: str) -> np.ndarray:
    lines = content.split('\n')
    lines_removed_double_space = [line.replace('  ', ' ') for line in lines]
    log_array = [line.split(' ') for line in lines_removed_double_space]
    log_array.pop() # Last line is empty
    data_start_index = find_data_start_index(log_array)

    if not data_start_index == None:
        log_array_np_ready = [[val for val in sublist if val] for sublist in log_array[data_start_index+1:]]
        data_np_array = np.array(log_array_np_ready)
        return  data_np_array
    else:
        print('Could not find start index of data logging')
        exit()


def get_array_for_each_file(logs_file_paths: list[str]) -> np.ndarray:
    array_of_logs = np.empty([1, len(logs_file_paths)], dtype=object)
    for index, log_file_path in enumerate(logs_file_paths):
        file_contents = get_log_contents(log_file_path)
        data_np_array = parse_content(file_contents)
        array_of_logs[0][index] = data_np_array
    return array_of_logs.squeeze()



def main():
    logs_file_directory = 'ramp/*.txt'
    logs_file_paths = get_logs(logs_file_directory)
    # The rows of the logs differ based on the length of the testrun
    array_of_all_logs = get_array_for_each_file(logs_file_paths)
    
    index_x = 7
    index_y = 8
    
    for log in array_of_all_logs:
        x1 = log[:,index_x].astype(np.float16)
        y1 = log[:,index_y].astype(np.float16)
        plt.plot(x1,y1)
    plt.show()

if __name__ == "__main__":
    main()