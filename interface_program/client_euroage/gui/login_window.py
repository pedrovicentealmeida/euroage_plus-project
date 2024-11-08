from tkinter import Label, LabelFrame, Entry, Button, NORMAL, DISABLED, messagebox
from network.client import SocketClient

class LoginWindow:
    def __init__(self, root, client: SocketClient, on_login_success):
        self.root = root
        self.client = client
        self.on_login_success = on_login_success
        
        self.root.geometry('500x300')

        txt_dados = Label(root, text="Credenciais de Acesso", font=("Consolas", 16), anchor="center", padx=5)
    
        txt_login = LabelFrame(root, text="Login: ", font=("Consolas", 14), padx=5)
        self.entry_login = Entry(txt_login, width=35, font=("Consolas", 13))
        
        txt_pass = LabelFrame(root, text="Password: ", font=("Consolas", 14), padx=5)
        self.entry_pass = Entry(txt_pass, width=35, font=("Consolas", 13), show="*")

        self.entry_login.bind("<KeyRelease>", self.enable_login_button)
        self.entry_pass.bind("<KeyRelease>", self.enable_login_button)
        
        self.btn_login = Button(root, text="Iniciar sessão", font=("Consolas", 13), borderwidth=1, command=self.check_login, state=DISABLED)
        
        txt_dados.pack(pady=15)
        
        txt_login.pack(pady=5)
        self.entry_login.pack(pady=3)
        
        txt_pass.pack(pady=10)
        self.entry_pass.pack(pady=3)
        
        self.btn_login.pack(pady=10)

    def enable_login_button(self, event=None):
        if self.entry_login.get() and self.entry_pass.get():
            self.btn_login.config(state=NORMAL)
        else:
            self.btn_login.config(state=DISABLED)
    
    def check_login(self):
        self.client.send_message(self.entry_login.get())
        self.client.send_message(self.entry_pass.get())

        data =  self.client.receive_message()
        
        if data == "1":
            login_successful = True

            if login_successful:
                self.on_login_success()
            
            #if robot_socket_text is None:
            #    messagebox.showerror("Erro!", "Sem connexão com o robô!")
            #    connectivity()
        
        else:
            messagebox.showwarning("Aviso!", "Credenciais erradas")

        
