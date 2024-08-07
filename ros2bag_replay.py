import subprocess

def play_rosbag(bag_path):
    try:
        subprocess.run(['ros2', 'bag', 'play', bag_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f'Error occurred while playing the bag file: {e}')

if __name__ == '__main__':
    # 記録されたrosbag2ファイルのディレクトリを指定
    bag_path = '/path/to/your/recorded_bag_directory'
    play_rosbag(bag_path)
