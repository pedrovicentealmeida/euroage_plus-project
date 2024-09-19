import socket
import sqlite3

con = sqlite3.connect("mydatabase.db")
cur = con.cursor()

HOST = '0.0.0.0'
PORT = 3000

global client_socket

def start_connection():
    global client_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    
    server_socket.listen()
    
    client_socket, client_address = server_socket.accept() 
    print(f"Connection from {client_address} established.")
    
    return(client_socket)

def send_client(text):
    global client_socket
    client_socket.sendall(text.encode())
    client_socket.recv(5) # Confirmação de chegada

def recive_client():
    global client_socket
    data = client_socket.recv(1024).decode()
    client_socket.sendall("1".encode())
    return (data) 

def check_login():
    global client_socket

    while True:
        login = recive_client()
        password = recive_client()

        cur.execute("SELECT * FROM Institutions WHERE email=?", (login,))
        institution = cur.fetchone()

        if institution is not None: # Verifica que há correspondência para o login
            if institution[2] == password:
                send_client("1")
                send_client(institution[3]) # IP do robo
                return institution[0]

            send_client("0") # Pass errada
        else:
            send_client("-1") # Login não detetados

def get_players(institution_id):

    cur.execute("SELECT * FROM Patients WHERE institution_id = ?", (institution_id,))
    patients = cur.fetchall()

    send_client(str(len(patients)))

    # Send each patient's data
    for patient in patients:
        send_client(str(patient[0]))  # Send ID
        send_client(str(patient[2]))  # Send name
        send_client(str(patient[3]))  # Send age
        send_client(str(patient[4]))  # Send mmse
        send_client(str(patient[5]))  # Send mmse_text
        send_client(str(patient[6]))  # Send profession
        send_client(str(patient[7]))  # Send hobbies
        send_client(str(patient[8]))  # Send names_relations

def save_player(institution_id):

    global client_socket

    name = recive_client()
    age = int(recive_client())
    profession = recive_client()
    hobbies = recive_client()
    mmse_scale = int(recive_client())
    mmse_scale_text = recive_client()
    nomes_relacoes = recive_client()

    cur.execute("""
        INSERT INTO Patients (institution_id, name, age, mmse, mmse_text, profession, hobbies, names_relations)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    """, (institution_id, name, age, mmse_scale, mmse_scale_text, profession, hobbies, nomes_relacoes))

    con.commit()

    send_client("0")

def edit_player(institution_id):
    player_id = int(recive_client())
    
    name = recive_client()
    age = int(recive_client())
    profession = recive_client()
    hobbies = recive_client()
    mmse_scale = int(recive_client())
    mmse_scale_text = recive_client()
    nomes_relacoes = recive_client()

    cur.execute("""
        UPDATE Patients
        SET name = ?, age = ?, mmse = ?, mmse_text = ?, profession = ?, hobbies = ?, names_relations = ?
        WHERE id = ? AND institution_id = ?
    """, (name, age, mmse_scale, mmse_scale_text, profession, hobbies, nomes_relacoes, player_id, institution_id))

    con.commit()

    send_client("0")

def delete_player(institution_id):
    player_id = int(recive_client()) # ID para remover da base de dados
    
    cur.execute("DELETE FROM Patients WHERE id = ? AND institution_id = ?", (player_id, institution_id))
    con.commit()    

def new_ip(institution_id):
    
    new_ip_address = recive_client()
    print(new_ip_address)
    
    cur.execute("""
        UPDATE Institutions
        SET robot_ip = ?
        WHERE id = ?
    """, (new_ip_address, institution_id))
    con.commit()
    
def main():
    global client_socket
    
    while True:
        client_socket = start_connection()
        ID_institution = check_login()

        while True:
            number_root = recive_client()
            print(number_root)
            
            if number_root == "100":
                get_players(ID_institution)
            
            elif number_root == "200":
                save_player(ID_institution)
            
            elif number_root == "300":
                edit_player(ID_institution)
            
            elif number_root == "400":
                delete_player(ID_institution)
            
            elif number_root == "600":
                new_ip(ID_institution)
                
            else:
                break

if __name__ == "__main__":
    cur.execute("""
        CREATE TABLE IF NOT EXISTS Institutions (
            id INTEGER PRIMARY KEY,
            email TEXT,
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

    #cur.execute("""
    #    INSERT INTO Institutions (email, password, robot_ip)
    #    VALUES ('caritas@gmail.com', 'qwerty', '127.0.0.1')
    #""")
    #cur.execute("SELECT * FROM Patients WHERE institution_id = ?", (1,))
    #patients = cur.fetchall()
    #print(patients)
    # Executar a instrução para deletar a tabela
    #cur.execute("DROP TABLE IF EXISTS Institutions")
    #cur.execute("DROP TABLE IF EXISTS Patients")
    #cur.execute("DELETE FROM Patients;")
    #cur.execute("DELETE FROM Institutions;")
    
    # Salvando as mudanças no banco de dados
    #con.commit()

    main()