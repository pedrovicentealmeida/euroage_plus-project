from network.client import SocketClientServer
from typing import List, Tuple
import tkinter as tk

class PatientDataFetcher:
    def __init__(self, client: SocketClientServer):
        self.client = client
        self.number_of_patients = 0
        self.data = []

    def fetch_patient_data(self):
        self.client.send_message_server("100")
        self.number_of_patients = int(self.client.receive_message_server())
        self.data = self.fetch_patient_details()
        return(self.data)

    def fetch_patient_details(self) -> List[Tuple]:
        data = []
        for _ in range(self.number_of_patients):
            id = int(self.client.receive_message_server())
            name = self.client.receive_message_server()
            age = int(self.client.receive_message_server())
            mmse = int(self.client.receive_message_server())
            mmse_text = self.client.receive_message_server()
            profession = self.client.receive_message_server()
            hobbies = self.client.receive_message_server()
            relations = self.client.receive_message_server()
            data.append((id, name, age, mmse, mmse_text, hobbies, profession, relations))
        return data
    
def clear_root(root: tk):
    for child in root.winfo_children():
        if isinstance(child, tk.Menu):
            continue 
        child.destroy()