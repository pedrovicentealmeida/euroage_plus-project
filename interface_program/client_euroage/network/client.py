import socket
from config import SERVER_HOST, SERVER_PORT, ROBOT_TEXT_PORT, ROBOT_COMMANDS_PORT
import time

class SocketClientServer:
    def __init__(self):
        self.server_host = SERVER_HOST
        self.server_port = SERVER_PORT
        self.client_server_socket = None

    def connect(self):
        try:
            self.client_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_server_socket.connect((self.server_host, self.server_port))
            return True
        except Exception as e:
            print(f"Erro ao conectar ao servidor: {e}")
            return False

    def send_message_server(self, message):
        try:
            self.client_server_socket.sendall(message.encode('utf-8'))
            # Aguarda o ACK do servidor antes de enviar outra mensagem
            ack = self.client_server_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK não recebido. Aguardando...")
        except Exception as e:
            print(f"Erro ao enviar mensagem: {e}")

    def receive_message_server(self):
        try:
            data = self.client_server_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao servidor para confirmar o recebimento
            self.client_server_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Erro ao receber mensagem do servidor.")
            return None

    def close(self):
        if self.client_server_socket:
            self.client_server_socket.close()

class SocketClientRobot:
    def __init__(self, robot_host):
        self.robot_host = robot_host
        self.robot_text_port = ROBOT_TEXT_PORT
        self.robot_commands_port = ROBOT_COMMANDS_PORT
        
        self.client_text_robot_socket = None
        self.client_commands_robot_socket = None

    def connect(self, robot_host):
        self.robot_host = robot_host
        try:
            self.client_text_robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_text_robot_socket.settimeout(2)
            self.client_text_robot_socket.connect((self.robot_host, self.robot_text_port))
            time.sleep(0.5)
            self.client_commands_robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_commands_robot_socket.settimeout(2) 
            self.client_commands_robot_socket.connect((self.robot_host, self.robot_commands_port))
            return True
        
        except socket.timeout:
            print(f"Erro: Conexão ao robô {self.robot_host} excedeu o tempo limite de 5 segundos.")
            self.client_text_robot_socket = None
            self.client_commands_robot_socket = None
            return False
        except ConnectionRefusedError as e:
            print(f"Connection refused: {e}")
            self.client_text_robot_socket = None
            self.client_commands_robot_socket = None
            return False
        except socket.error as e:
            print(f"Socket error: {e}")
            self.client_text_robot_socket = None
            self.client_commands_robot_socket = None
            return False

    def send_commands_robot(self, message):
        try:
            self.client_commands_robot_socket.sendall(message.encode('utf-8'))
            # Aguarda o ACK do servidor antes de enviar outra mensagem
            ack = self.client_commands_robot_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK não recebido. Aguardando...")
        except Exception as e:
            print(f"Erro ao enviar mensagem: {e}")

    def receive_commands_robot(self):
        try:
            data = self.client_commands_robot_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao servidor para confirmar o recebimento
            self.client_commands_robot_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Erro ao receber mensagem do servidor.")
            return None

    def receive_text_robot(self):
        try:
            data = self.client_text_robot_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao servidor para confirmar o recebimento
            self.client_text_robot_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            #print("Erro ao receber mensagem do servidor.")
            return None

    def close(self):
        if self.client_text_robot_socket:
            self.client_text_robot_socket.close()
        
        if self.client_commands_robot_socket:
            self.client_commands_robot_socket.close()
