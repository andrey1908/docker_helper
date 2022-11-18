from docker_helper import DockerContainer


class RosDockerContainer(DockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.ros_version = None

    def check_output(self, command):
        if self.ros_version:
            command = "source /opt/ros/{}/setup.bash && ".format(self.ros_version) + command
        output = super().check_output(command)
        return output

    def run_command(self, command, suppress_output=False):
        if self.ros_version:
            command = "source /opt/ros/{}/setup.bash && ".format(self.ros_version) + command
        return_code = super().run_command(command, suppress_output=suppress_output)
        return return_code

    def run_command_async(self, command, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        if self.ros_version:
            command = "source /opt/ros/{}/setup.bash && ".format(self.ros_version) + command
        super().run_command_async(command, session)

    def create_containter(self, volume_args):
        super().create_containter(volume_args)

        get_ros_version_command = "rosversion -d"
        self.ros_version = self.check_output(get_ros_version_command).replace('\n', '').replace('\r', '')

        set_ros_ip_command = "export ROS_IP={}".format(self.container_ip)
        return_code = self.run_command("echo '{}' >> ~/.bashrc".format(set_ros_ip_command))
        if return_code != 0:
            raise RuntimeError("Error adding ROS_IP to ~/.bashrc")

    def start_roscore(self, wait=True):
        self.run_command_async("roscore", 'roscore')
        if wait:
            command = "while : ; do rostopic list && break || sleep 0.1; done"
            return_code = self.run_command(command, suppress_output=True)
            if return_code != 0:
                raise RuntimeError("Error waiting for ros master")

    def connect_to_ros_master(self, ros_master_ip):
        connect_to_ros_master_command = "export ROS_MASTER_URI=http://{}:11311".format(ros_master_ip)
        return_code = self.run_command("echo '{}' >> ~/.bashrc".format(connect_to_ros_master_command))
        if return_code != 0:
            raise RuntimeError("Error adding ROS_MASTER_URI to ~/.bashrc to connect to ros master")

    def rosrun(self, package, executable, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "rosrun {} {} {}".format(package, executable, arguments)
        return_code = self.run_command(command)
        return return_code

    def roslaunch(self, package, launch, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "roslaunch {} {} {}".format(package, launch, arguments)
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError("Error running roslaunch:\n  {}".format(command))

    def roslaunch_nopkg(self, launch_filename, arguments='', source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "roslaunch {} {}".format(launch_filename, arguments)
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError("Error running roslaunch:\n  {}".format(command))

    def rosrun_async(self, package, executable, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "rosrun {} {} {}".format(package, executable, arguments)
        self.run_command_async(command, session)

    def roslaunch_async(self, package, launch, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "roslaunch {} {} {}".format(package, launch, arguments)
        self.run_command_async(command, session)

    def roslaunch_nopkg_async(self, launch_filename, arguments='', session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        command = ''.join("source {} && ".format(source_file) for source_file in source_files)
        command += "roslaunch {} {}".format(launch_filename, arguments)
        self.run_command_async(command, session)

    def rosbag_play(self, rosbag_filenames, arguments=''):
        if isinstance(rosbag_filenames, str):
            rosbag_filenames = [rosbag_filenames]
        return_code = self.rosrun("rosbag", "play", '--clock {} {}'.format(arguments, ' '.join(rosbag_filenames)))
        if return_code != 0:
            raise RuntimeError("Error playing rosbags")
            
    def use_sim_time(self, value):
        command = "rosparam set use_sim_time {}".format(str(value))
        return_code = self.run_command(command)
        if return_code != 0:
            raise RuntimeError("Error setting use_sim_time")

    def stop_roscore(self):
        self.stop_session('roscore')
