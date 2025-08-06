import socket
import struct

# 创建UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 绑定IP地址和端口
server_address = ('localhost', 43897)
server_socket.bind(server_address)

print('UDP server is listening...')

while True:
    # 接收数据
    data, address = server_socket.recvfrom(1024)
    received_int = struct.unpack('<i', data)[0]

    print(f"Received integer value: {received_int} from {address}")

# 关闭socket（这里无法执行到）
server_socket.close()
