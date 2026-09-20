import subprocess
import time


class SshTunnel:
    def __init__(self, host, local_port=18011, remote_port=8011, remote_host="127.0.0.1"):
        if not host.strip():
            raise ValueError("ssh host must not be empty")
        self.host = host
        self.local_port = int(local_port)
        self.remote_port = int(remote_port)
        self.remote_host = remote_host
        self.process = None

    def start(self, timeout_s=8.0):
        command = ["ssh", "-N", "-T", "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=15",
                   "-o", "ServerAliveCountMax=3", "-L", "%d:%s:%d" % (self.local_port, self.remote_host, self.remote_port), self.host]
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                error = self.process.stderr.read().decode("utf-8", "replace")
                raise RuntimeError("SSH tunnel failed: " + error.strip())
            time.sleep(0.05)
        return self

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_args):
        self.close()
