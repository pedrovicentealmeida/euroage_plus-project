from tkinter import LabelFrame, Label, PhotoImage, StringVar, Button, messagebox, Entry, Toplevel
from network.client import SocketClientServer, SocketClientRobot
from gui_components import CustomToolbar
import config
import re

class ConnectivityWindow:
    def __init__(self, root, server: SocketClientServer, robot: SocketClientRobot, toolbar: CustomToolbar):
        self.root = root
        self.server = server
        self.robot = robot
        self.toolbar = toolbar
        self.create_gui_elements()
    
    def create_gui_elements(self):
        main_frame = LabelFrame(self.root, text="Conexões: ", font=("Consolas", 20), padx=20, pady=20)
        main_frame.pack(pady=25, padx=25, fill="both", expand=True)

        self.create_server_frame(main_frame)
        nota_frame_server = Label(self.root, font=("Consolas", 10), text="Nota: Ligação estabelecida com o servidor do ISR da Universidade de Coimbra que funciona como banco de dados de todos os pacientes!", anchor='w')
        nota_frame_server.place(x=85, y=286)

        self.create_robot_frame(main_frame)
        nota_frame_robot = Label(self.root, font=("Consolas", 10), text="Nota: Ligação entre este software e o robô! Possível necessidade de alteração de endereço de IP!", anchor='w')
        nota_frame_robot.place(x=85, y=555)

        Label(main_frame, text="", font=("Consolas", 1)).pack(pady=10)

    def create_server_frame(self, parent):
        self.root.imagem_green_1 = PhotoImage(file="icons/green.png")

        server_frame = LabelFrame(parent, text="Servidor: ", font=("Consolas", 18), padx=20, pady=20)

        ip_frame_server = Label(server_frame, font=("Consolas", 15), text="Domínio: ISR UC", anchor='w')
        status_frame_server = Label(server_frame, font=("Consolas", 15), text="Estado: Conectado", anchor='w')  
        image_frame_server = Label(server_frame, image=self.root.imagem_green_1)
        
        server_frame.pack(pady=10, fill="x")
        ip_frame_server.grid(row=0, column=0, sticky='w', pady=25, padx=15)
        status_frame_server.grid(row=1, column=0, sticky='w', pady=25, padx=15)
        image_frame_server.grid(row=1, column=1, pady=5, padx=5)

    def create_robot_frame(self, parent):
        self.root.imagem_red_1 = PhotoImage(file="icons/red.png")
        self.root.imagem_green = PhotoImage(file="icons/green.png")
        
        robot_frame = LabelFrame(parent, text="Robô: ", font=("Consolas", 18), padx=20, pady=20)
        
        robot_ip_text = StringVar()
        robot_ip_text.set("Endereço IP: " + config.ROBOT_HOST)
        ip_frame_robot = Label(robot_frame, font=("Consolas", 15), textvariable=robot_ip_text, anchor='w')
        
        self.status_frame_robot = Label(robot_frame, font=("Consolas", 15), text="Estado: Conectado", anchor='w')
        
        if self.robot.client_commands_robot_socket is None or self.robot.client_text_robot_socket is None:
            self.status_frame_robot.config(text="Estado: Desconectado")
            self.image_frame_robot = Label(robot_frame, image=self.root.imagem_red_1)
        
        else:
            self.status_frame_robot.config(text="Estado: Conectado")
            self.image_frame_robot = Label(robot_frame, image=self.root.imagem_green_1)
            
        self.root.imagem_edit = PhotoImage(file="icons/edit.png")
        self.root.imagem_refresh = PhotoImage(file="icons/refresh.png")
        
        edit_button = Button(robot_frame, image=self.root.imagem_edit, command=lambda ip_var=robot_ip_text: self.edit_ip(ip_var), font=("Consolas", 12), borderwidth=0)
        refresh_button = Button(robot_frame, image=self.root.imagem_refresh, command=self.refresh_connection, font=("Consolas", 12), borderwidth=0)
        
        robot_frame.pack(pady=30, fill="x")
        ip_frame_robot.grid(row=0, column=0, sticky='w', pady=25, padx=15)
        self.status_frame_robot.grid(row=1, column=0, sticky='w', pady=25, padx=15)
        self.image_frame_robot.grid(row=1, column=1, pady=5, padx=5)
        edit_button.grid(row=0, column=1, columnspan=2, pady=5, padx=5)
        refresh_button.grid(row=0, column=2, columnspan=2, pady=5, padx=50)

    def refresh_connection(self):
        if self.status_frame_robot.cget("text") == "Estado: Desconectado":
                self.robot.close()
            
                if self.robot.connect(config.ROBOT_HOST):
                    self.toolbar.change_ip(self.robot)
                    self.status_frame_robot.config(text="Estado: Conectado")
                    self.toolbar.update_status_icon(True)
                    self.image_frame_robot.config(image=self.root.imagem_green_1)
                else:
                    self.robot.client_commands_robot_socket = None
                    self.robot.client_text_robot_socket = None
                    self.status_frame_robot.config(text="Estado: Desconectado")
                    self.toolbar.update_status_icon(False)
                    self.image_frame_robot.config(image=self.root.imagem_red_1)
                    messagebox.showwarning("Aviso!", "IP sem conexão")

        else:
            messagebox.showwarning("Aviso!", "Conexão bem estabelecida!")
    
    def edit_ip(self, ip_variable):        
        def validate_ip(*args):
            new_ip = ip_entry_var.get()
            ip_pattern = re.compile(r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$')

            if ip_pattern.match(new_ip):
                parts = new_ip.split('.')
                valid_parts = all(0 <= int(part) <= 255 for part in parts)
                if valid_parts:
                    confirm_button.config(state='normal')
                else:
                    confirm_button.config(state='disabled')
            else:
                confirm_button.config(state='disabled')

        def on_confirm():
            self.server.send_message_server("600")

            config.ROBOT_HOST = ip_entry_var.get()            
            self.server.send_message_server(config.ROBOT_HOST)
            ip_variable.set("Endereço IP: " + config.ROBOT_HOST)

            if self.robot.client_commands_robot_socket or self.robot.client_text_robot_socket is not None:
                self.robot.close()
            
            if self.robot.connect(config.ROBOT_HOST):
                self.toolbar.change_ip(self.robot)
                self.status_frame_robot.config(text="Estado: Conectado")
                self.toolbar.update_status_icon(True)
                self.image_frame_robot.config(image=self.root.imagem_green_1)
            else:
                self.robot.client_commands_robot_socket = None
                self.robot.client_text_robot_socket = None
                self.status_frame_robot.config(text="Estado: Desconectado")
                self.image_frame_robot.config(image=self.root.imagem_red_1)
                self.toolbar.update_status_icon(False)
                messagebox.showwarning("Aviso!", "IP sem conexão")
            
            dialog.destroy()

        dialog = Toplevel(self.root)
        dialog.title("Novo IP")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()

        Label(dialog, text="Insira o novo IP do robô:", font=("Consolas", 13)).pack(pady=10)
        
        ip_entry_var = StringVar()
        ip_entry_var.trace("w", validate_ip)
        
        ip_entry = Entry(dialog, textvariable=ip_entry_var, font=("Consolas", 13))
        ip_entry.pack(pady=5)
        
        confirm_button = Button(dialog, text="Confirmar", command=on_confirm, font=("Consolas", 13), state='disabled')
        confirm_button.pack(pady=10)