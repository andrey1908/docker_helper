import subprocess
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
        volume = '{}:{}:rw'.format(host_common_path, docker_mount_folder)
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
        volume = '{}:{}:rw'.format(host_common_path, docker_mount_folder)
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
            docker_file, volume = DockerMounts.pass_files_to_docker(host_file, '/mnt/docker_mounts_{}'.format(len(self.mounted)))
            self.mounted[host_file] = docker_file
            self.volume_args = self.volume_args + '-v {} '.format(volume)

        for host_folder in self.host_folders:
            if host_folder is None:
                continue
            if isinstance(host_folder, list):
                host_folder = tuple(host_folder)
            docker_folder, volume = DockerMounts.pass_folders_to_docker(host_folder, '/mnt/docker_mounts_{}'.format(len(self.mounted)))
            self.mounted[host_folder] = docker_folder
            self.volume_args = self.volume_args + '-v {} '.format(volume)

    def __getitem__(self, key):
        if isinstance(key, list):
            key = tuple(key)
        return self.mounted.get(key, None)


class DockerContainer:
    def __init__(self, image_name, container_name, user_name=None):
        self.image_name = image_name
        self.container_name = container_name
        self.user_name = user_name
        self.user_arg = '--user {}'.format(user_name) if user_name else ''

    def create_containter(self, docker_mounts: DockerMounts):
        docker_command = "docker run -it -d --rm --privileged --name {} " \
            "--env DISPLAY={} --env QT_X11_NO_MITSHM=1 " \
            "--ipc host --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all " \
            "-v /tmp/.X11-unix:/tmp/.X11-unix:rw {} {}".format(self.container_name, os.environ['DISPLAY'], docker_mounts.volume_args, self.image_name)
        return_code = subprocess.call(docker_command.split())
        if return_code != 0:
            raise RuntimeError("Error creating docker container")

        get_container_ip_command = "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' " + self.container_name
        container_ip = subprocess.check_output(get_container_ip_command.split())
        self.container_ip = container_ip.decode('utf-8').replace('\n', '').replace('\r', '')[1:-1]
        
        get_home_directory_command = "cd ~; pwd"
        self.home_directory = self.check_output(get_home_directory_command).replace('\n', '').replace('\r', '')

    def check_output(self, command):
        docker_command = "docker exec -it {} {} /bin/bash -c".format(self.user_arg, self.container_name)
        output = subprocess.check_output("{} '{}'".format(docker_command, command), shell=True)
        output = output.decode('utf-8')
        return output

    def run_command(self, command, suppress_output=False):
        if suppress_output:
            stdout = subprocess.DEVNULL
        else:
            stdout = None
        docker_command = "docker exec -it {} {} /bin/bash -ic".format(self.user_arg, self.container_name)
        return_code = subprocess.call(docker_command.split() + [command], stdout=stdout)
        return return_code

    def run_command_async(self, command, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        async_command = "tmux new -d -s {} /bin/bash -c '{}'".format(session, command)
        docker_command = "docker exec -it {} {} /bin/bash -ic".format(self.user_arg, self.container_name)
        return_code = subprocess.call(docker_command.split() + [async_command], stdout = subprocess.DEVNULL)
        if return_code != 0:
            raise RuntimeError("Error running command in async mode:\n  {}".format(command))

    def stop_session(self, session):
        stop_command = "(tmux send-keys -t ={}: C-c) && (tmux a -t ={} || true)".format(session, session)
        docker_command = "docker exec -it {} {} /bin/bash -c".format(self.user_arg, self.container_name)
        return_code = subprocess.call(docker_command.split() + [stop_command])
        if return_code != 0:
            raise RuntimeError("Error stopping session '{}'".format(session))
        return  # TODO: return exit status from tmux session

    def stop_container(self):
        docker_command = "docker stop {}".format(self.container_name)
        return_code = subprocess.call(docker_command.split())
        if return_code != 0:
            raise RuntimeError("Error stopping docker container")
