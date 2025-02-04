import socket

# Listen on port 8880 (camera)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 8880))

print("Listening for UDP packets on port 8880...")
while True:
    data, addr = sock.recvfrom(65535)
    print(f"Received from {addr}: {len(data)} bytes")
