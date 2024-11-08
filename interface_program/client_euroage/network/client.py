import socket
from config import SERVER_HOST, SERVER_PORT

class SocketClient:
    def __init__(self):
        self.server_host = SERVER_HOST
        self.server_port = SERVER_PORT
        self.client_socket = None
        self.connect()

    def connect(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_host, self.server_port))

    def send_message(self, message):
        try:
            self.client_socket.sendall(message.encode('utf-8'))
            # Aguarda o ACK do servidor antes de enviar outra mensagem
            ack = self.client_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK não recebido. Aguardando...")
        except Exception as e:
            print(f"Erro ao enviar mensagem: {e}")

    def receive_message(self):
        try:
            data = self.client_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao servidor para confirmar o recebimento
            self.client_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Erro ao receber mensagem do servidor.")
            return None

    def close(self):
        if self.client_socket:
            self.client_socket.close()
