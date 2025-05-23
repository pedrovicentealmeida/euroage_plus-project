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
    
    def save_player(self, institution_id, name, age, mmse_scale, mmse_scale_text, profession, hobbies, names_relations):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("""
                INSERT INTO Patients (institution_id, name, age, mmse, mmse_text, profession, hobbies, names_relations)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (institution_id, name, age, mmse_scale, mmse_scale_text, profession, hobbies, names_relations))

            con.commit()
    
    def edit_player(self, player_id, name, age, profession, hobbies, mmse_scale, mmse_scale_text, names_relations):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("""
                UPDATE Patients
                SET name = ?, age = ?, profession = ?, hobbies = ?, mmse = ?, mmse_text = ?, names_relations = ?
                WHERE id = ?
            """, (name, age, profession, hobbies, mmse_scale, mmse_scale_text, names_relations, player_id))
            
            con.commit()
        
    def delete_player(self, player_id):
        with self.get_connection() as con:
            cur = con.cursor()
            cur.execute("DELETE FROM Patients WHERE id = ?", (player_id,))
            con.commit()

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.database = Database()
        
        self.server_socket = None
        self.client_socket = None

    def send_message(self, message):
        try:
            self.client_socket.sendall(message.encode('utf-8'))
            # Aguarda o ACK do cliente antes de enviar a próxima mensagem
            ack = self.client_socket.recv(1024).decode('utf-8')
            if ack != "ACK":
                print("ACK não recebido. Aguardando...")
        except BrokenPipeError:
            print("Erro: Conexão quebrada - cliente desconectado.")
            self.client_socket.close()

    def receive_message(self):
        try:
            data = self.client_socket.recv(1024).decode('utf-8')
            # Envia um ACK de volta ao cliente para confirmar o recebimento
            self.client_socket.sendall("ACK".encode('utf-8'))
            return data
        except socket.error:
            print("Erro ao receber mensagem do cliente.")
            self.client_socket.close()
            return None

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print("Server listening on port", self.port)

    def check_login(self):
        login = self.receive_message()
        password = self.receive_message()

        if login and password:
            institution = self.database.validate_login(login, password)
            if institution:
                self.send_message("1")  # Login success
                if institution[3] == "":
                    self.send_message("127.0.0.1")
                else:
                    self.send_message(institution[3])
                return institution[0]  # Return institution ID
            else:
                self.send_message("0" if institution else "-1")
        else:
            self.send_message("-1")  # Missing login or password
        
        return None

    def handle_client(self, address):
        try:
            print(f"Connection from {address} established.")

            while True:
                institution_id = self.check_login()
            
                if institution_id:
                    self.interface_loop(institution_id)
                    break
                else:
                    print("Failed login attempt from", address)
            
            self.client_socket.close()
        
        except Exception as e:
            print(f"Error handling client {address}: {e}")
            self.client_socket.close()

    def interface_loop(self, institution_id):
        while True:
            command = self.receive_message()
            
            if command == "100":
                print("Executing get_players for institution:", institution_id)
                patients = self.database.get_patients(institution_id)
                self.send_patients(patients)
            
            elif command == "200":
                print("Executing save_player for institution:", institution_id)
                self.save_player(institution_id)
            
            elif command == "300":
                print("Executing edit_player for institution:", institution_id)
                self.edit_player()

            elif command == "400":
                print("Executing delete_player for institution:", institution_id)
                player_id = int(self.receive_message())
                self.database.delete_player(player_id)
            
            elif command == "600":
                print("Executing edit_ip for institution: ", institution_id)
                new_ip = self.receive_message()
                print("Request to change IP for: ", new_ip)
                self.database.edit_ip(new_ip, institution_id)
                
            elif command is None:
                print("Client disconnected.")
                break
            else:
                print("Unknown command received:", command)

    def send_patients(self, patients):
        self.send_message(str(len(patients)))

        for patient in patients:
            # patient is a tuple: (id, institution_id, name, age, mmse, mmse_text, profession, hobbies, names_relations)
            self.send_message(str(patient[0]))  # Send ID
            self.send_message(str(patient[2]))  # Send name
            self.send_message(str(patient[3]))  # Send age
            self.send_message(str(patient[4]))  # Send mmse
            self.send_message(str(patient[5]))  # Send mmse_text
            self.send_message(str(patient[6]))  # Send profession
            self.send_message(str(patient[7]))  # Send hobbies
            self.send_message(str(patient[8]))  # Send names_relations

    def save_player(self, institution_id):

        name = self.receive_message()
        age = int(self.receive_message())
        profession = self.receive_message()
        hobbies = self.receive_message()
        mmse_scale = int(self.receive_message())
        mmse_scale_text = self.receive_message()
        nomes_relacoes = self.receive_message()

        self.database.save_player(institution_id, name, age, mmse_scale, mmse_scale_text, profession, hobbies, nomes_relacoes)

        self.send_message("0") # Success message

    def edit_player(self):
        player_id = int(self.receive_message())
    
        name = self.receive_message()
        age = int(self.receive_message())
        profession = self.receive_message()
        hobbies = self.receive_message()
        mmse_scale = int(self.receive_message())
        mmse_scale_text = self.receive_message()
        nomes_relacoes = self.receive_message()

        self.database.edit_player(player_id, name, age, profession, hobbies, mmse_scale, mmse_scale_text, nomes_relacoes)

        self.send_message("0") # Success message

    def run(self):
        self.start_server()
        try:
            while True:
                self.client_socket, client_address = self.server_socket.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(client_address,))
                client_thread.daemon = True  # Allows server to exit even if threads are running
                client_thread.start()
        except KeyboardInterrupt:
            print("Server shutting down.")
        finally:
            self.server_socket.close()

if __name__ == "__main__":
    server = Server(HOST, PORT)
    server.run()
