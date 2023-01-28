import subprocess_tee
import os
import os.path as osp


class DockerMounts:
    def __init__(self, files=tuple(), folders=tuple()):
        self.host_files = files
        self.host_folders = folders

        self.mounted = dict()
        self.volume_args = ''
        self.pass_to_docker()

    @staticmethod
    def pass_files_to_docker(host_files, docker_mount_folder):
        single_file = False
        if isinstance(host_files, str):
            host_files = [host_files]
            single_file = True
        host_filenames = list(map(lambda file: osp.abspath(osp.expanduser(file)), host_files))
        host_common_path = osp.commonpath(list(map(lambda filename: osp.dirname(filename), host_filenames)))
        docker_files = list(map(lambda host_filename: osp.relpath(host_filename, host_common_path), host_filenames))
        docker_filenames = list(map(lambda docker_file: osp.join(docker_mount_folder, docker_file), docker_files))
        volume = f'{host_common_path}:{docker_mount_folder}:rw'
        if single_file:
            return docker_filenames[0], volume
        else:
            return docker_filenames, volume

    @staticmethod
    def pass_folders_to_docker(host_folders, docker_mount_folder):
        single_folder = False
        if isinstance(host_folders, str):
            host_folders = [host_folders]
            single_folder = True
        host_absolute_folders = list(map(lambda host_folder: osp.abspath(osp.expanduser(host_folder)), host_folders))
        host_common_path = osp.commonpath(host_absolute_folders)
        docker_folders = list(map(lambda host_absolute_folder: osp.relpath(host_absolute_folder, host_common_path), host_absolute_folders))
        docker_absolute_folders = list(map(lambda docker_folder: osp.normpath(osp.join(docker_mount_folder, docker_folder)), docker_folders))
        volume = f'{host_common_path}:{docker_mount_folder}:rw'
        if single_folder:
            return docker_absolute_folders[0], volume
        else:
            return docker_absolute_folders, volume

    def pass_to_docker(self):
        for host_file in self.host_files:
            if host_file is None:
                continue
            if isinstance(host_file, list):
                host_file = tuple(host_file)
            docker_file, volume = DockerMounts.pass_files_to_docker(host_file, f'/mnt/mounted_{len(self.mounted)}')
            self.mounted[host_file] = docker_file
            self.volume_args = self.volume_args + f'-v {volume} '

        for host_folder in self.host_folders:
            if host_folder is None:
                continue
            if isinstance(host_folder, list):
                host_folder = tuple(host_folder)
            docker_folder, volume = DockerMounts.pass_folders_to_docker(host_folder, f'/mnt/mounted_{len(self.mounted)}')
            self.mounted[host_folder] = docker_folder
            self.volume_args = self.volume_args + f'-v {volume} '

    def __getitem__(self, key):
        if isinstance(key, list):
            key = tuple(key)
        return self.mounted.get(key, None)


class DockerContainer:
    def __init__(self, image_name, container_name, user_name=None):
        self.image_name = image_name
        self.container_name = container_name
        self.user_name = user_name
        self.user_arg = f'--user {user_name}' if user_name else ''

    def create_containter(self, mounts: DockerMounts=None, net='host'):
        if mounts is None:
            mounts = DockerMounts()
        docker_command = f"docker run -it -d --rm --privileged --name {self.container_name} " \
            f"--env DISPLAY={os.environ.get('DISPLAY', '')} --env QT_X11_NO_MITSHM=1 " \
            f"--ipc host --net {net} --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all " \
            f"-v /tmp/.X11-unix:/tmp/.X11-unix:rw {mounts.volume_args} {self.image_name}"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        # Could not create container. Maybe it is already running.
        if returncode != 0:
            check_container_running_command = f"[ \"$(docker ps | grep {self.container_name})\" ]"
            container_running = (subprocess_tee.run(check_container_running_command, quiet=True).returncode == 0)
            if container_running:
                print("Container is already running")
            else:
                raise RuntimeError(f"Could not create or find already running container '{self.container_name}'")

        get_container_ip_command = "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' " + self.container_name
        self.container_ip = subprocess_tee.run(get_container_ip_command, quiet=True).stdout.rstrip()

        suppress_redundant_output_command = "touch ~/.sudo_as_admin_successful"
        self.run(suppress_redundant_output_command, quiet=True)

        get_home_directory_command = "cd ~; pwd"
        self.home_directory = self.run(get_home_directory_command, quiet=True).stdout.rstrip()

    def run(self, command: str, quiet=False):
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_arg} {self.container_name} /bin/bash -ic $'{command}'"
        result = subprocess_tee.run(docker_command, quiet=quiet)
        return result

    def run_async(self, command: str, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        async_command = f"tmux new -d -s {session} /bin/bash -c $'{command}'"
        async_command = async_command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_arg} {self.container_name} /bin/bash -ic $'{async_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error running command in async mode:\n  {command}")

    def stop_session(self, session: str):
        stop_command = f"(tmux send-keys -t ={session}: C-c) && (tmux a -t ={session} || true)"
        docker_command = f"docker exec -it {self.user_arg} {self.container_name} /bin/bash -ic '{stop_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error stopping session '{session}'")
        return  # TODO: return exit status from tmux session

    def stop_container(self):
        docker_command = f"docker stop {self.container_name}"
        returncode = subprocess_tee.run(docker_command).returncode
        if returncode != 0:
            raise RuntimeError("Error stopping docker container")
