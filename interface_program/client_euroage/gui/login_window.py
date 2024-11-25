from tkinter import Label, LabelFrame, Entry, Button, NORMAL, DISABLED, messagebox
from network.client import SocketClientServer

class LoginWindow:

    def __init__(self, root, server: SocketClientServer):
        self.root = root
        self.server = server
        self.login_check = False
        self.create_gui_elements()

    def create_gui_elements(self):
        txt_dados = Label(self.root, text="Credenciais de Acesso", font=("Consolas", 16), anchor="center", padx=5)
        txt_dados.pack(pady=15)
        self.define_componnets()

    def define_componnets(self):
        txt_login = LabelFrame(self.root, text="Login: ", font=("Consolas", 14), padx=5)
        self.entry_login = Entry(txt_login, width=35, font=("Consolas", 13))

        txt_login.pack(pady=5)
        self.entry_login.pack(pady=3)
        
        txt_pass = LabelFrame(self.root, text="Password: ", font=("Consolas", 14), padx=5)
        self.entry_pass = Entry(txt_pass, width=35, font=("Consolas", 13), show="*")

        txt_pass.pack(pady=10)
        self.entry_pass.pack(pady=3)

        self.entry_login.bind("<KeyRelease>", self.enable_login_button)
        self.entry_pass.bind("<KeyRelease>", self.enable_login_button)
        
        self.btn_login = Button(self.root, text="Iniciar sessão", font=("Consolas", 13), borderwidth=1, command=self.check_login, state=DISABLED)
        self.btn_login.pack(pady=10)

    def enable_login_button(self, event=None):
        if self.entry_login.get() and self.entry_pass.get():
            self.btn_login.config(state=NORMAL)
        else:
            self.btn_login.config(state=DISABLED)
    
    def check_login(self):
        self.server.send_message_server(self.entry_login.get())
        self.server.send_message_server(self.entry_pass.get())

        data =  self.server.receive_message_server()
        
        if data == "1":
            self.login_check = True

        else:
            messagebox.showwarning("Aviso!", "Credenciais erradas")