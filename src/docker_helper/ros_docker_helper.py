from .docker_helper import DockerMounts, DockerContainer


class RosDockerContainer(DockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.ros_version = None

    def create_containter(self, net='host', docker_mounts: DockerMounts = None):
        super().create_containter(net, docker_mounts)

        get_ros_version_command = "rosversion -d"
        self.ros_version = self.check_output(get_ros_version_command).replace('\n', '').replace('\r', '')

        set_ros_ip_command = f"export ROS_IP={self.container_ip}"
        return_code = self.run_command(f"echo '{set_ros_ip_command}' >> ~/.bashrc")
        if return_code != 0:
            raise RuntimeError("Error adding ROS_IP to ~/.bashrc")

    def check_output(self, command):
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        output = super().check_output(command)
        return output

    def run_command(self, command, suppress_output=False):
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        return_code = super().run_command(command, suppress_output=suppress_output)
        return return_code

    def run_command_async(self, command, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        super().run_command_async(command, session)

    def start_roscore(self, wait=True):
        self.run_command_async("roscore", 'roscore')
        if wait:
            command = "while : ; do rostopic list && break || sleep 0.1; done"
            return_code = self.run_command(command, suppress_output=True)
            if return_code != 0:
                raise RuntimeError("Error waiting for ros master")

    def connect_to_ros_master(self, ros_master_ip):
        connect_to_ros_master_command = f"export ROS_MASTER_URI=http://{ros_master_ip}:11311"
        return_code = self.run_command(f"echo '{connect_to_ros_master_command}' >> ~/.bashrc")
        if return_code != 0:
            raise RuntimeError("Error adding ROS_MASTER_URI to ~/.bashrc to connect to ros master")

    def rosrun(self, package, executable, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        return_code = self.run_command(command)
        return return_code

    def roslaunch(self, package, launch, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError(f"Error running roslaunch:\n  {command}")

    def roslaunch_nopkg(self, launch_filename, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_filename} {arguments}"
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError(f"Error running roslaunch:\n  {command}")

    def rosrun_async(self, package, executable, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        self.run_command_async(command, session)

    def roslaunch_async(self, package, launch, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        self.run_command_async(command, session)

    def roslaunch_nopkg_async(self, launch_filename, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_filename} {arguments}"
        self.run_command_async(command, session)

    def rosbag_play(self, rosbag_filenames, arguments=''):
        if isinstance(rosbag_filenames, str):
            rosbag_filenames = [rosbag_filenames]
        return_code = self.rosrun("rosbag", "play", f"--clock {arguments} {' '.join(rosbag_filenames)}")
        if return_code != 0:
            raise RuntimeError("Error playing rosbags")
            
    def use_sim_time(self, value):
        command = f"rosparam set use_sim_time {str(value)}"
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError("Error setting use_sim_time")

    def stop_roscore(self):
        self.stop_session('roscore')
