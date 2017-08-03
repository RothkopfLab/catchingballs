import socket
import sys

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_message(message):
    try:
        s.sendto(message + '\n', ('WV', 7070))
    except:
        pass
        


if __name__ == "__main__":
    send_message('Test')
