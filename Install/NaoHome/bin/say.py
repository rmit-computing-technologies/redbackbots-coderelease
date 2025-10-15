#!/usr/bin/python3
import socket
import threading
import os
import json
import heapq
import time
import logging
import subprocess
import signal


# How old a message needs to be for say to drop it
SAY_TIMEOUTS = {
    'DEBUG': 1,
    'INFO': 2,
    'WARNING': 3,
    'ERROR': 5,
    'CRITICAL': 8,
    'SOUND': 10,
}

LOG_LEVELS = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARNING': logging.WARNING,
    'ERROR': logging.ERROR,
    'CRITICAL': logging.CRITICAL,
    'SOUND' : logging.NOTSET,
}

SOUNDS_PATH = "/home/nao/config/Sounds"

# Params to use with espeak, see espeak --help for more info
# Comment out the value key to revert to default
ESPEAK_PARAMS = {
    # Amplitude, 0 to 200, default is 100
    "volume": {
        "option": "a",
        # "value": 200,
    },
    # Use voice file of this name from espeak-data/voices
    "voice_file": {
        "option": "v",
        "value": "f2",
        "default": "m1"
    },
    # Pitch adjustment, 0 to 99, default is 50
    "pitch": {
        "option": "p",
        "value": 70
    },
    # Speed in approximate words per minute. The default is 175
    "speed": {
        "option": "s",
        "value": 155
    },
    # Word gap. Pause between words, units of 10mS at the default speed
    "word_gap": {
        "option": "g",
        "value": 6,
    }
}

# Turn all set espeak params into cli arg string
ESPEAK_PARAMS_STRING = " ".join(
    [
        f"-{param['option']}{param['value']}"
        for param 
        in ESPEAK_PARAMS.values() if 'value' in param
    ]
)

class Say:
    """
    Class to handle the say functionality. It will receive messages from the specified port
    and add to the say queue (which is a priority queue). The say queue will be processed
    with the most critical messages first, and the oldest messages will be dropped if they
    exceed the specified timeouts in SAY_TIMEOUTS
    """
    def __init__(self, host='localhost', port=65432):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.say_queue = []
        try:
            self.play_sound("startup.wav")
        except FileNotFoundError:
            print("espeak is not installed or not found in the system path. Skipping initialization message.")
            print("Initialised say.py")
            self.play_sound("sirene.wav")
        except Exception as e:
            print(f"An error occurred while trying to use eSpeak: {e}")

    def play_sound(self, sound: str):
        """
        Play a sound file from the SOUNDS_PATH directory
        """
        try:
            os.system(f"/usr/bin/aplay {SOUNDS_PATH}/{sound}")
        except FileNotFoundError:
            print(f"Sound file {sound} not found in {SOUNDS_PATH}")
        except Exception as e:
            print(f"An error occurred while trying to play sound file {sound}: {e}")

    def say_messages(self):
        """
        This function will be run in a separate thread and will process the say queue.
        It will use the espeak command to say the messages in the queue.
        Any messages too old will be dropped according to SAY_TIMEOUTS
        """
        while True:
            current_time = time.time()
            if self.say_queue:
                _, log_time, data = heapq.heappop(self.say_queue)
                if current_time - log_time > SAY_TIMEOUTS[data['level']]:
                    # If the message is too old, skip it, but play a sound so we know
                    print((f"msg: {data} was too old to say | {current_time - log_time} > {SAY_TIMEOUTS[data['level']]}"))
                    self.say_queue.clear()
                    self.play_sound("beep.wav")
                    continue
                message = data["message"]
                try:
                    subprocess.check_call(f"espeak {ESPEAK_PARAMS_STRING} '{message}'", shell=True)
                except FileNotFoundError:
                    print("espeak is not installed or not found in the system path. Skipping initialization message.")
                except Exception as e:
                    print(f"An error occurred while trying to use eSpeak: {e}")
            else:
                time.sleep(0.1)

    def handle_client(self, client_socket, addr):
        """
        This function will be run in a separate thread for each client connection.
        It will receive messages from the daemon, behaviours or C++ add them to the say queue.
        """
        print(f"Connection from {addr}")
        while True:
            payload = client_socket.recv(1024).decode('utf-8')
            if not payload:
                print(f"Connection closed by {addr}")
                break
            try:
                data = json.loads(payload)
                print(f"Received from {addr}: {data}")
                
                if data["level"] == "SOUND":
                    # If the level is SOUND, treat the message as a sound file name in sounds
                    sound_file = data["message"]
                    self.play_sound(sound_file)
                else:
                    # Otherwise, add the message to the say queue
                    heapq.heappush(self.say_queue, (-LOG_LEVELS[data["level"]], time.time(), data))
                
                client_socket.sendall(b"ACK")
            except json.JSONDecodeError as e:
                print(f"Invalid JSON from {addr}: {payload} - {e}")
                client_socket.sendall(b"ERR")
            except socket.error as e:
                print(f"Socket error from {addr}: {e}")
        client_socket.close()

    def start(self):
        """
        This function will start the say service and listen for connections.
        It will also start the message handling thread.
        """
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            print(f"Service listening on {self.host}:{self.port}")

            # Start the message handling thread
            threading.Thread(target=self.say_messages, daemon=True).start()

            while True:
                try:
                    client_socket, addr = self.server_socket.accept()
                    client_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                    client_thread.start()
                except OSError as e:
                    print(f"Error accepting connection: {e}")
                    break
        except OSError as e:
            print(f"Error binding to {self.host}:{self.port}: {e}")
        finally:
            self.server_socket.close()
            print("Say Socket closed")


if __name__ == "__main__":
    say_thread = Say()
    say_thread.start()
