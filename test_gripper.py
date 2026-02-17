import socket
import time

ROBOT_IP = "192.168.X.X"   # <<< PUT THE IP FROM THE PENDANT
PORT = 30002

def send(cmd):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_IP, PORT))
    s.send((cmd + "\n").encode("utf-8"))
    s.close()

print("Activating gripper...")
send(
"def prog():\n"
"  rq_activate()\n"
"end\n"
"prog()"
)

time.sleep(2)

print("Closing...")
send(
"def prog():\n"
"  rq_close()\n"
"end\n"
"prog()"
)

time.sleep(3)

print("Opening...")
send(
"def prog():\n"
"  rq_open()\n"
"end\n"
"prog()"
)

print("Done.")
