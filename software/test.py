import socket

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
message = b"test"
udp_socket.sendto(message, ('127.0.0.1', 4211))
udp_socket.close()