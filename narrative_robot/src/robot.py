#!/usr/bin/env python3

import socket
import threading
import time
import rclpy

rclpy.init(args=None)
from game_library import StoryTelling

ST = StoryTelling()

HOST = '0.0.0.0'
PORT_TEXT = 3001
PORT_COMMANDS = 3002

global client_socket_text
global client_socket_commands

global stop
global pause

def start_connection_text():
    global client_socket_text
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT_TEXT))
    
    server_socket.listen()
    
    client_socket_text, client_address = server_socket.accept() 
    print(f"Connection from {client_address} established.")
    
    return(client_socket_text)

def start_connection_commands():
    global client_socket_commands
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT_COMMANDS))
    
    server_socket.listen()
    
    client_socket_commands, client_address = server_socket.accept() 
    print(f"Connection from {client_address} established.")
    
    return(client_socket_commands)

def send_client(text):
    global client_socket_text
    client_socket_text.sendall(text.encode())
    print(client_socket_text.recv(5))

def receive_client():
    global client_socket_text
    data = client_socket_text.recv(1024).decode()
    print(data)
    client_socket_text.sendall("-".encode())
    return (data) 

def receive_client_commands():
    global client_socket_commands
    data = client_socket_commands.recv(1024).decode()
    print(data)
    client_socket_commands.sendall("-".encode())
    return (data) 
  
def new_story():
    theme = receive_client()
    topics = receive_client()
    
    player_name = receive_client()
    player_age = int(receive_client())
    player_mmse = receive_client()
    player_hobbies = receive_client()
    player_profession = receive_client()
    player_relations = receive_client()
    
    ST.define_parameters(player_name, player_age, player_mmse, player_hobbies, player_profession, player_relations, theme, topics)
    
    wait_for_start = receive_client_commands()
    
    def receive_commands():
        global stop
        global pause
        
        while True:
            text = receive_client_commands()
            if text:
                if text == "-1":
                    stop = True
                    pause = False
                    return
                elif text == "0":
                    pause = True
                elif text == "1":
                    pause = False
    
    if wait_for_start == "1":
        global stop
        global pause
        
        stop = False
        pause = False
        
        receive_commands_thread = threading.Thread(target=receive_commands)
        receive_commands_thread.start()
        
        while True:
            try:
                response = ST.obtain_response()
                send_client("Robô: " + response)
                
                if response.endswith("Fim.") or stop:
                    send_client("\nRobô: " + response)
                    send_client("-1")
                    return
                
                if pause:
                    send_client("YES")
                    while pause:
                        time.sleep(1)
                
                while True:
                    if stop:
                        send_client("-1")
                        return
                    
                    text = ST.new_message()
                    
                    if text and not stop and not pause:
                        send_client("\nPaciente: " + text)
                        break
                    
                    elif stop:
                        send_client("-1")
                        return
                    
                    if pause:
                        text=""
                        send_client("YES")
                        
                        while pause:
                            time.sleep(1)
                    
            except Exception as e:
                send_client(f"Error: {e}")
                break
  
def main():

    global client_socket_text
    global client_socket_commands
    
    try:
        while True:
            client_socket_text = start_connection_text()
            client_socket_commands = start_connection_commands()

            while True:
                number_root = receive_client()
                print(number_root)
                
                if number_root == "500":
                    new_story()
                elif number_root == "CLOSE":
                    client_socket_text.close()
                    client_socket_commands.close()
                    break
    finally:
        # Cleanup on exit
        ST.shutdown_hook()
        ST.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()