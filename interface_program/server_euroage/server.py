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

import socket
import sqlite3
import threading
from contextlib import contextmanager

HOST = '0.0.0.0'
PORT = 3000

class Database:
    def __init__(self, db_name="mydatabase.db"):
        self.db_name = db_name
        self.setup_database()

    def setup_database(self):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("""
                CREATE TABLE IF NOT EXISTS Institutions (
                    id INTEGER PRIMARY KEY,
                    email TEXT UNIQUE,
                    password TEXT,
                    robot_ip TEXT
                );
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS Patients (
                    id INTEGER PRIMARY KEY,
                    institution_id INTEGER,
                    name TEXT,
                    age INTEGER,
                    mmse INTEGER,
                    mmse_text TEXT, 
                    profession TEXT,
                    hobbies TEXT,
                    names_relations TEXT,
                    FOREIGN KEY (institution_id) REFERENCES Institutions(id)
                );
            """)

    @contextmanager
    def get_connection(self):
        con = sqlite3.connect(self.db_name)
        try:
            yield con
        finally:
            con.close()

    def validate_login(self, email, password):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("SELECT * FROM Institutions WHERE email=?", (email,))
            institution = cur.fetchone()
            if institution and institution[2] == password:
                return institution
            return None

    def get_patients(self, institution_id):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("SELECT * FROM Patients WHERE institution_id = ?", (institution_id,))
            return cur.fetchall()
    
    def edit_ip(self, new_ip_address, institution_id):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("""
                UPDATE Institutions
                SET robot_ip = ?
                WHERE id = ?
            """, (new_ip_address, institution_id))
            con.commit()

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.database = Database()
        self.server_socket = None

    def send_message(self, client_socket, message):
        try:
            client_socket.sendall(message.encode('utf-8'))
            # Aguarda o ACK do cliente antes de enviar a próxima mensagem
            ack = client_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK não recebido. Aguardando...")
        except BrokenPipeError:
            print("Erro: Conexão quebrada - cliente desconectado.")
            client_socket.close()

    def receive_message(self, client_socket):
        try:
            data = client_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao cliente para confirmar o recebimento
            client_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Erro ao receber mensagem do cliente.")
            client_socket.close()
            return None

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print("Server listening on port", self.port)

    def check_login(self, client_socket):
        login = self.receive_message(client_socket)
        password = self.receive_message(client_socket)

        if login and password:
            institution = self.database.validate_login(login, password)
            if institution:
                self.send_message(client_socket, "1")  # Login success
                if institution[3] == "":
                    self.send_message(client_socket, "127.0.0.1")
                else:
                    self.send_message(client_socket, institution[3])
                return institution[0]  # Return institution ID
            else:
                self.send_message(client_socket, "0" if institution else "-1")
        else:
            self.send_message(client_socket, "-1")  # Missing login or password
        
        return None

    def handle_client(self, client_socket, address):
        try:
            print(f"Connection from {address} established.")

            while True:
                institution_id = self.check_login(client_socket)
            
                if institution_id:
                    self.interface_loop(client_socket, institution_id)
                    break
                else:
                    print("Failed login attempt from", address)
            
            client_socket.close()
        
        except Exception as e:
            print(f"Error handling client {address}: {e}")
            client_socket.close()

    def interface_loop(self, client_socket, institution_id):
        while True:
            command = self.receive_message(client_socket)
            if command == "100":
                print("Executing get_players for institution:", institution_id)
                patients = self.database.get_patients(institution_id)
                self.send_patients(patients, client_socket)
            
            elif command == "600":
                print("Executing edit_ip for institution: ", institution_id)
                new_ip = self.receive_message(client_socket)
                print("Request to change IP for: ", new_ip)
                self.database.edit_ip(new_ip, institution_id)
                
            elif command is None:
                print("Client disconnected.")
                break
            else:
                print("Unknown command received:", command)

    def send_patients(self, patients, client_socket):
        self.send_message(client_socket, str(len(patients)))

        for patient in patients:
            # patient is a tuple: (id, institution_id, name, age, mmse, mmse_text, profession, hobbies, names_relations)
            self.send_message(client_socket, str(patient[0]))  # Send ID
            self.send_message(client_socket, str(patient[2]))  # Send name
            self.send_message(client_socket, str(patient[3]))  # Send age
            self.send_message(client_socket, str(patient[4]))  # Send mmse
            self.send_message(client_socket, str(patient[5]))  # Send mmse_text
            self.send_message(client_socket, str(patient[6]))  # Send profession
            self.send_message(client_socket, str(patient[7]))  # Send hobbies
            self.send_message(client_socket, str(patient[8]))  # Send names_relations

    def run(self):
        self.start_server()
        try:
            while True:
                client_socket, client_address = self.server_socket.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
                client_thread.daemon = True  # Allows server to exit even if threads are running
                client_thread.start()
        except KeyboardInterrupt:
            print("Server shutting down.")
        finally:
            self.server_socket.close()

if __name__ == "__main__":
    server = Server(HOST, PORT)
    server.run()
