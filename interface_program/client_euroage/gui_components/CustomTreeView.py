from tkinter import ttk

class CustomTreeView:
    def __init__(self, parent):
        self.tree = ttk.Treeview(parent, columns=("id", "name", "age", "mmse", "mmse_text", "hobbies", "profession", "names_friends"), show="headings")
        self.apply_styles()
        self.configure_columns()
        
    def apply_styles(self):
        style = ttk.Style()
        style.configure("Treeview.Heading", font=("Consolas", 12))
        style.configure("Treeview", font=("Consolas", 10), rowheight=25)
        
    def configure_columns(self):
        self.tree.column("id", width=25, anchor="center")
        self.tree.column("name", width=150, anchor="center")
        self.tree.column("age", width=75, anchor="center")
        self.tree.column("mmse", width=50, anchor="center")
        self.tree.column("mmse_text", width=185, anchor="center")
        self.tree.column("hobbies", width=280, anchor="center")
        self.tree.column("profession", width=200, anchor="center")
        self.tree.column("names_friends", width=250, anchor="center")

        self.tree.heading("id", text="ID")
        self.tree.heading("name", text="Nome")
        self.tree.heading("age", text="Idade")
        self.tree.heading("mmse", text="MMSE")
        self.tree.heading("mmse_text", text="Défict Cognitivo")
        self.tree.heading("hobbies", text="Passatempos")
        self.tree.heading("profession", text="Profissão Passada")
        self.tree.heading("names_friends", text="Familiares e Amigos")

        self.tree.pack(fill="both", expand=True)
        
    def insert_data(self, data):
        for item in data:
            self.tree.insert("", "end", values=item)
