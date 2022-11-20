from .docker_helper import DockerMounts, DockerContainer


class RosDockerContainer(DockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.ros_version = None

    def create_containter(self, net='host', docker_mounts: DockerMounts=None):
        super().create_containter(net=net, docker_mounts=docker_mounts)

        get_ros_version_command = "rosversion -d"
        self.ros_version = self.run(get_ros_version_command, quiet=True).stdout.rstrip()

        if self.container_ip:
            set_ros_ip_command = f"export ROS_IP={self.container_ip}"
            returncode = self.run(f"echo '{set_ros_ip_command}' >> ~/.bashrc", quiet=True).returncode
            if returncode != 0:
                raise RuntimeError("Error adding ROS_IP to ~/.bashrc")

    def run(self, command, quiet=False):
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        result = super().run(command, quiet=quiet)
        return result

    def run_async(self, command, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        super().run_async(command, session=session)

    def start_roscore(self, wait=True):
        self.run_async("roscore", 'roscore')
        if wait:
            command = "while : ; do rostopic list && break || sleep 0.1; done"
            returncode = self.run(command, quiet=True).returncode
            if returncode != 0:
                raise RuntimeError("Error waiting for ros master")

    def connect_to_ros_master(self, ros_master_ip):
        connect_to_ros_master_command = f"export ROS_MASTER_URI=http://{ros_master_ip}:11311"
        returncode = self.run(f"echo '{connect_to_ros_master_command}' >> ~/.bashrc").returncode
        if returncode != 0:
            raise RuntimeError("Error adding ROS_MASTER_URI to ~/.bashrc to connect to ros master")

    def rosrun(self, package, executable, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        result = self.run(command)
        return result

    def roslaunch(self, package, launch, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        returncode = self.run(command).returncode
        if returncode != 0:
            raise RuntimeError(f"Error running roslaunch:\n  {command}")

    def roslaunch_nopkg(self, launch_filename, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_filename} {arguments}"
        returncode = self.run(command).returncode
        if returncode != 0:
            raise RuntimeError(f"Error running roslaunch:\n  {command}")

    def rosrun_async(self, package, executable, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        self.run_async(command, session)

    def roslaunch_async(self, package, launch, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        self.run_async(command, session)

    def roslaunch_nopkg_async(self, launch_filename, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_filename} {arguments}"
        self.run_async(command, session)

    def rosbag_play(self, rosbag_filenames, arguments=''):
        if isinstance(rosbag_filenames, str):
            rosbag_filenames = [rosbag_filenames]
        returncode = self.rosrun("rosbag", "play", f"--clock {arguments} {' '.join(rosbag_filenames)}").returncode
        if returncode != 0:
            raise RuntimeError("Error playing rosbags")
            
    def use_sim_time(self, value):
        command = f"rosparam set use_sim_time {str(value)}"
        returncode = self.run(command).returncode
        if returncode != 0:
            raise RuntimeError("Error setting use_sim_time")

    def stop_roscore(self):
        self.stop_session('roscore')
