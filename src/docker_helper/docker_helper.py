import subprocess_tee
import os
import os.path as osp
from copy import deepcopy


class DockerMounts:
    def __init__(self):
        self.mounts_counter = 0
        self.mounts = dict()  # host_folder -> docker_folder
        self.is_mount_pinned = dict()  # host_folder -> bool
        self.modes = dict()  # host_folder -> str

    def add_file(self, host_file, docker_destination_folder=None, mode='rw'):
        host_folder = osp.dirname(host_file)
        self.add_folder(host_folder, docker_destination_folder=docker_destination_folder, mode=mode)

    def add_folder(self, host_folder, docker_destination_folder=None, mode='rw'):
        host_folder = osp.realpath(osp.expanduser(host_folder))
        if host_folder in self.mounts:
            raise RuntimeError(f"Folder {host_folder} already mounted to {self.mounts[host_folder]}")
        if docker_destination_folder is None:
            docker_destination_folder = f"/mnt/mount_{self.mounts_counter}"
            self.mounts_counter += 1
            mount_pinned = False
        else:
            docker_destination_folder = osp.normpath(docker_destination_folder)
            mount_pinned = True

        self.mounts[host_folder] = docker_destination_folder
        self.is_mount_pinned[host_folder] = mount_pinned
        self.modes[host_folder] = mode
        self._trim_mounts()

    def _trim_mounts(self):
        host_folders_to_remove = set()
        for i, host_folder in enumerate(self.mounts.keys()):
            for j, host_sub_folder in enumerate(self.mounts.keys()):
                if i == j:
                    continue
                if osp.commonpath([host_folder, host_sub_folder]) != host_sub_folder:
                    continue
                if self.is_mount_pinned[host_folder]:
                    continue
                host_folders_to_remove.add(host_folder)

        for host_folder_to_remove in host_folders_to_remove:
            self.mounts.pop(host_folder_to_remove)
            self.is_mount_pinned.pop(host_folder_to_remove)
            self.modes.pop(host_folder_to_remove)

    def resolve(self, host_path):
        host_path = osp.realpath(osp.expanduser(host_path))
        host_folder = None
        for host_folder_to_check in self.mounts.keys():
            if osp.commonpath([host_folder_to_check, host_path]) == host_folder_to_check:
                if host_folder is None:
                    host_folder = host_folder_to_check
                else:
                    if self.is_mount_pinned[host_folder_to_check] and not self.is_mount_pinned[host_folder]:
                        host_folder = host_folder_to_check
                    elif not self.is_mount_pinned[host_folder_to_check] and self.is_mount_pinned[host_folder]:
                        continue
                    elif len(host_folder_to_check) > len(host_folder):
                        host_folder = host_folder_to_check
        if host_folder is None:
            return None

        resolved_path = osp.join(self.mounts[host_folder], osp.relpath(host_path, host_folder))
        resolved_path = osp.normpath(resolved_path)
        return resolved_path

    def get_mount_arguments(self):
        mount_arguments = str()
        for host_folder, docker_folder in self.mounts.items():
            mount_arguments += f"-v {host_folder}:{docker_folder}:{self.modes[host_folder]} "
        return mount_arguments


class DockerContainer:
    def __init__(self, image_name, container_name, user_name=None):
        self.image_name = image_name
        self.container_name = container_name
        self.user_name = user_name

        self.user_argument = f'--user {user_name}' if user_name else ''
        self.mounts = DockerMounts()

        self.default_mounts = DockerMounts()
        self.default_mounts.add_folder("/tmp/.X11-unix", "/tmp/.X11-unix", "rw")
        self.default_mounts.add_folder("/etc/timezone", "/etc/timezone", "ro")
        self.default_mounts.add_folder("/etc/localtime", "/etc/localtime", "ro")

    def create_containter(self, mounts: DockerMounts=None, net='host'):
        if mounts is None:
            mounts = DockerMounts()
        docker_command = f"docker run -it -d --rm --privileged --name {self.container_name} " \
            f"--env DISPLAY={os.environ.get('DISPLAY', '')} --env QT_X11_NO_MITSHM=1 " \
            f"--ipc host --net {net} --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all " \
            f"{self.default_mounts.get_mount_arguments()} " \
            f"{mounts.get_mount_arguments()} " \
            f"{self.image_name}"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode

        # Could not create container. Maybe it is already running.
        if returncode != 0:
            check_container_running_command = f"[ \"$(docker ps | grep {self.container_name})\" ]"
            container_running = (subprocess_tee.run(check_container_running_command, quiet=True).returncode == 0)
            if container_running:
                print("Container is already running")
                get_mounts_command = "docker inspect -f '{{ json .Mounts }}' " + self.container_name
                true = True
                false = False
                mounts_list = eval(subprocess_tee.run(get_mounts_command, quiet=True).stdout.rstrip())
                self.mounts = DockerMounts()
                for mount_entry in mounts_list:
                    if mount_entry['Source'] in self.default_mounts.mounts:
                        continue
                    assert mount_entry['Type'] == 'bind'
                    self.mounts.add_folder(mount_entry['Source'], mount_entry['Destination'], mount_entry['Mode'])
                container_created = False
            else:
                raise RuntimeError(f"Could not create or find already running container '{self.container_name}'")
        else:
            self.mounts = deepcopy(mounts)
            container_created = True

        get_container_ip_command = "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' " + self.container_name
        self.container_ip = subprocess_tee.run(get_container_ip_command, quiet=True).stdout.rstrip()

        suppress_redundant_output_command = "touch ~/.sudo_as_admin_successful"
        self.run(suppress_redundant_output_command, quiet=True)

        get_home_directory_command = "cd ~; pwd"
        self.home_directory = self.run(get_home_directory_command, quiet=True).stdout.rstrip()

        return container_created

    def run(self, command: str, quiet=False):
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic $'{command}'"
        result = subprocess_tee.run(docker_command, quiet=quiet)
        return result

    def run_async(self, command: str, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        async_command = f"tmux new -d -s {session} /bin/bash -c $'{command}'"
        async_command = async_command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic $'{async_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error running command in async mode:\n  {command}")

    def stop_session(self, session: str):
        stop_command = f"(tmux send-keys -t ={session}: C-c) && (tmux a -t ={session} || true)"
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic '{stop_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error stopping session '{session}'")
        return  # TODO: return exit status from tmux session

    def stop_container(self):
        docker_command = f"docker stop {self.container_name}"
        returncode = subprocess_tee.run(docker_command).returncode
        if returncode != 0:
            raise RuntimeError("Error stopping docker container")

    def resolve(self, host_path):
        resolved_path = self.mounts.resolve(host_path)
        if resolved_path is None:
            raise RuntimeError(f"Could not resolve path '{host_path}'")
        return resolved_path
