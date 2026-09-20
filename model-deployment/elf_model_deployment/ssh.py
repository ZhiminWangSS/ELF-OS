import subprocess
import time


class SshTunnel:
    def __init__(self, host, local_port=18012, remote_port=8012):
        if not str(host).strip():
            raise ValueError("ssh host must not be empty")
        self.host, self.local_port, self.remote_port = host, int(local_port), int(remote_port)
        self.process = None

    def start(self, timeout_s=8.0):
        command = ["ssh", "-N", "-T", "-o", "ExitOnForwardFailure=yes",
                   "-o", "StrictHostKeyChecking=yes",
                   "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
                   "-L", "%d:127.0.0.1:%d" % (self.local_port, self.remote_port), self.host]
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("SSH tunnel failed: " + self.process.stderr.read().decode("utf-8", "replace"))
            time.sleep(.05)
        return self

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None
