##################################################################################
# BSD 3-Clause License
# 
# Copyright (c) 2025, Pedro V. Almeida
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################

#!/usr/bin/env python3

import socket
import threading
import rclpy
import time
from game_library import StoryTelling

rclpy.init(args=None)

ST = StoryTelling()

ros_thread = threading.Thread(target=rclpy.spin, args=(ST,))
ros_thread.start()

HOST = '0.0.0.0'
PORT_TEXT = 3005
PORT_COMMANDS = 3010

class Robot:
    def __init__(self, host, port_text, port_commands):
        self.host = host
        self.port_text = port_text
        self.port_commands = port_commands
        
        self.robot_text_socket = None
        self.robot_commands_socket = None
        
        self.start = False
        self.pause = False
        self.stop = False
    
    def start_robot_text(self):
        """Starts the server socket to receive text and continuously accepts connections."""
        self.robot_text_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_text_socket.bind((self.host, self.port_text))
        self.robot_text_socket.listen()
        print(f"Robot text listening on port {self.port_text}", flush=True)
        
        while True:
            client_socket, client_address = self.robot_text_socket.accept()
            print(f"Text connection from {client_address}", flush=True)
            # Start a new thread for each client
            threading.Thread(target=self.handle_text_client, args=(client_socket,), daemon=True).start()

    def start_robot_commands(self):
        """Starts the server socket to receive commands and continuously accepts connections."""
        self.robot_commands_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_commands_socket.bind((self.host, self.port_commands))
        self.robot_commands_socket.listen()
        print(f"Robot commands listening on port {self.port_commands}", flush=True)
        
        while True:
            client_socket, client_address = self.robot_commands_socket.accept()
            print(f"Commands connection from {client_address}", flush=True)
            # Start a new thread for each client
            threading.Thread(target=self.handle_commands_client, args=(client_socket,), daemon=True).start()
    
    def send_text_client(self, client_socket, message):
        try:
            client_socket.sendall(message.encode('utf-8'))
            # Wait for ACK from the client before sending the next message
            ack = client_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK not received. Waiting...")
        except BrokenPipeError:
            print("Error: Broken connection - client disconnected.")

    def send_commands_client(self, client_socket, message):
        try:
            client_socket.sendall(message.encode('utf-8'))
            # Wait for ACK from the client before sending the next message
            ack = client_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK not received. Waiting...")
        except BrokenPipeError:
            print("Error: Broken connection - client disconnected.")
    
    def receive_commands_client(self, client_socket):
        try:
            data = client_socket.recv(1024).decode('utf-8')
            client_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Error receiving message from client.")
            return None

    def handle_text_client(self, client_socket):
        """Envia texto para o cliente e lida com desconexões de forma robusta."""
        try:
            while True:
                if self.start == True:
                    self.tell_story(client_socket)
                else:
                    time.sleep(1)
        except (ConnectionResetError, BrokenPipeError):
            print("Text client disconnected.")
        finally:
            client_socket.close()

    def handle_commands_client(self, client_socket):
        """Recebe comandos do cliente e lida com desconexões de forma robusta."""

        try:
            while True:
                number_root = self.receive_commands_client(client_socket)
                print(number_root, flush=True)

                if number_root == "500":
                    self.new_story(client_socket)

                    while True:
                        text = self.receive_commands_client(client_socket)
                        if text:
                            if text == "-1":
                                self.stop = True
                                self.pause = False
                                return
                            elif text == "0":
                                self.pause = True
                            elif text == "1":
                                self.pause = False
                elif number_root is None:
                    print("Client disconnected.", flush=True)
                    break
                else:
                    print("Unknown root received:", number_root, flush=True)
        except (ConnectionResetError, BrokenPipeError):
            print("Commands client disconnected.")
        finally:
            client_socket.close()

    def tell_story(self, client_socket):
        while True:    
            try:
                response = ST.obtain_response()
                self.send_text_client(client_socket, "Robô: " + response)
                
                if response.endswith("Fim.") or response.endswith("FIM.") or self.stop:
                    self.send_text_client(client_socket, "\nRobô: " + response)
                    self.send_text_client(client_socket, "-1")
                    self.start = False
                    return
                
                if self.pause:
                    self.send_text_client(client_socket, "YES")
                    while self.pause:
                        time.sleep(1)
                
                while True:
                    if self.stop:
                        self.send_text_client(client_socket, "-1")
                        self.start = False
                        return
                    
                    text = ST.new_message()
                    
                    if text and not self.stop and not self.pause:
                        self.send_text_client(client_socket, "\nPaciente: " + text)
                        break
                    
                    elif self.stop:
                        self.send_text_client(client_socket, "-1")
                        self.start = False
                        return
                    
                    if self.pause:
                        text=""
                        self.send_text_client(client_socket, "YES")
                        
                        while self.pause:
                            time.sleep(1)
                        
            except Exception as e:
                self.send_text_client(f"Error: {e}")
                break

    def recive_parameters(self, client_socket):
        theme = self.receive_commands_client(client_socket)
        topics = self.receive_commands_client(client_socket)
        
        player_name = self.receive_commands_client(client_socket)
        player_age = int(self.receive_commands_client(client_socket))
        player_mmse = self.receive_commands_client(client_socket)
        player_hobbies = self.receive_commands_client(client_socket)
        player_profession = self.receive_commands_client(client_socket)
        player_relations = self.receive_commands_client(client_socket)
        
        ST.define_parameters(player_name, player_age, player_mmse, player_hobbies, player_profession, player_relations, theme, topics)

    def new_story(self, client_socket):
        self.recive_parameters(client_socket)
        self.receive_commands_client(client_socket) # button start
        self.start = True

    def run(self):
        """Inicializa as threads de texto e comandos."""
        text_thread = threading.Thread(target=self.start_robot_text, daemon=True)
        commands_thread = threading.Thread(target=self.start_robot_commands, daemon=True)

        text_thread.start()
        commands_thread.start()

        text_thread.join()
        commands_thread.join()

if __name__ == "__main__":
    robot = Robot(HOST, PORT_TEXT, PORT_COMMANDS)
    robot.run()
