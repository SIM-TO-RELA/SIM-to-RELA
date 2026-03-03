import socket
import time

class RobotiqHandE_URCapSocket:
    """
    Controls a Robotiq gripper via the Robotiq URCap socket server (commonly port 63352).
    Works while ur_robot_driver is running because it uses a separate port.
    """

    def __init__(self, robot_ip: str, port: int = 63352, timeout: float = 2.0):
        self.robot_ip = robot_ip
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.robot_ip, self.port))

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _send(self, msg: str):
        if not self.sock:
            raise RuntimeError("Not connected. Call connect() first.")
        # URCap expects ASCII lines
        data = (msg.strip() + "\n").encode("ascii")
        self.sock.sendall(data)

    def activate(self, speed: int = 150, force: int = 100):
        # Typical activation sequence for URCap socket protocol
        self._send("SET ACT 1")
        time.sleep(0.1)
        self._send(f"SET SPE {max(0, min(speed, 255))}")
        self._send(f"SET FOR {max(0, min(force, 255))}")
        time.sleep(0.2)

    def open(self):
        # POS 0 = open for most Robotiq grippers via this protocol
        self._send("SET POS 0")
        self._send("SET GTO 1")

    def close_grip(self):
        # POS 255 = close for most Robotiq grippers via this protocol
        self._send("SET POS 255")
        self._send("SET GTO 1")


if __name__ == "__main__":
    ROBOT_IP = "192.168.56.101"  # <-- change this
    g = RobotiqHandE_URCapSocket(ROBOT_IP)

    g.connect()
    # g.activate(speed=200, force=150)
    # g.open()
    time.sleep(1.0)
    g.close_grip()
    time.sleep(1.0)
    g.open()
    # g.close()
