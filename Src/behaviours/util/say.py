import socket
import json

SAY_HOST = 'localhost'
SAY_PORT = 65432

def _send_socket_message(payload):
    """
    Send a message via socket and return response
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SAY_HOST, SAY_PORT))
        client_socket.sendall(json.dumps(payload).encode('utf-8'))
        return client_socket.recv(1024)

def say_payload(payload):
    """
    Send a payload to the say service
    format of payload:
    {
        "message": "Hello World",
        "level": "CRITICAL"
    }
    """
    _send_socket_message(payload)

def say(message):
    """
    Send a message to the say service
    """
    payload = {
        "message": message,
        "level": 'CRITICAL'
    }
    _send_socket_message(payload)
