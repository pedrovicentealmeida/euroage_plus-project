from network.client import SocketClient
from typing import List, Tuple
import tkinter as tk

class PatientDataFetcher:
    def __init__(self, client: SocketClient):
        self.client = client
        self.number_of_patients = 0
        self.data = []

    def fetch_patient_data(self):
        self.client.send_message("100")
        self.number_of_patients = int(self.client.receive_message())
        self.data = self.fetch_patient_details()
        return(self.data)

    def fetch_patient_details(self) -> List[Tuple]:
        data = []
        for _ in range(self.number_of_patients):
            id = int(self.client.receive_message())
            name = self.client.receive_message()
            age = int(self.client.receive_message())
            mmse = int(self.client.receive_message())
            mmse_text = self.client.receive_message()
            profession = self.client.receive_message()
            hobbies = self.client.receive_message()
            relations = self.client.receive_message()
            data.append((id, name, age, mmse, mmse_text, hobbies, profession, relations))
        return data
    
def clear_root(root: tk):
    for child in root.winfo_children():
        if isinstance(child, tk.Menu):
            continue 
        child.destroy()