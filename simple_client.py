import socket
COLOR_RED = '\033[91m'
COLOR_RESET = '\033[0m'
host = 'localhost'
port = 1234
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))
client_socket.settimeout(1)  # Adjust the timeout value as needed

while True:
    try:
        message = input(COLOR_RED + ">> " + COLOR_RESET)
        if not message:
            continue

        client_socket.send(message.encode())
        response = client_socket.recv(1024).decode()
        print(response)

    except socket.timeout:
        print("Response timeout reached.")