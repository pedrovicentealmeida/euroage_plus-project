import socket
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import threading
import time
import re

HOST_SERVER = '127.0.0.1'
HOST_ROBOT = ''

PORT_SERVER = 3000
PORT_ROBOT_TEXT = 3001
PORT_ROBOT_COMMANDS = 3002

global server_socket
global robot_socket_text
global robot_socket_commands
global previous_root

def connection_server():
    def verify_connection():
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.connect((HOST_SERVER, PORT_SERVER))
            return server_socket
        except ConnectionRefusedError:
            messagebox.showerror("Erro!", "Sem conexão!\nVerifique ligação à VPN.")
            return None

    global server_socket
    server_socket = verify_connection()
    return server_socket

def connection_robot_commands():
    def verify_connection():
        global HOST_ROBOT        
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.connect((HOST_ROBOT, PORT_ROBOT_COMMANDS))
            return server_socket
        except ConnectionRefusedError:
            return None

    global robot_socket_commands
    robot_socket_commands = verify_connection()
    return robot_socket_commands

def connection_robot_text():
    def verify_connection():
        global HOST_ROBOT        
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.connect((HOST_ROBOT, PORT_ROBOT_TEXT))
            return server_socket
        except ConnectionRefusedError:
            return None

    global robot_socket_text
    robot_socket_text = verify_connection()
    return robot_socket_text

def send_server(text):
    global server_socket
    server_socket.sendall(text.encode())
    server_socket.recv(5)

def receive_server():
    global server_socket
    data = server_socket.recv(1024).decode()
    server_socket.sendall("-".encode())
    return (data) 

def send_robot(text):
    global robot_socket_text
    robot_socket_text.sendall(text.encode())
    robot_socket_text.recv(5)

def receive_robot():
    global robot_socket_text
    data = robot_socket_text.recv(1024).decode()
    robot_socket_text.sendall("-".encode())
    return (data)     

def send_robot_commands(text):
    global robot_socket_commands
    robot_socket_commands.sendall(text.encode())
    robot_socket_commands.recv(5)

def login():
    
    global server_socket
    
    txt_dados = Label(root, text="Credenciais de Acesso", font=("Consolas", 16), anchor="center", padx=5)
    
    txt_login = LabelFrame(text="Login: ", font=("Consolas", 14), padx=5)
    entry_login = Entry(txt_login, width=35, font=("Consolas", 13))
    
    txt_pass = LabelFrame(text="Password: ", font=("Consolas", 14), padx=5)
    entry_pass = Entry(txt_pass, width=35, font=("Consolas", 13), show="*")
    
    def check_login():
        send_server(entry_login.get())
        send_server(entry_pass.get())
        
        data =  receive_server()
        
        if data == "1":
            root.geometry('1300x700')
            global HOST_ROBOT
            HOST_ROBOT = receive_server()
            new_story()
            
            if robot_socket_text is None:
                messagebox.showerror("Erro!", "Sem connexão com o robô!")
                connectivity()
            
        elif data == "0":
            messagebox.showwarning("Aviso!", "Password Errada")
        
        else:
            messagebox.showwarning("Aviso!", "Login Inválido")
    
    def enable_login_button(event=None):
        if entry_login.get() and entry_pass.get():
            btn_login.config(state=NORMAL)
        else:
            btn_login.config(state=DISABLED)
    
    entry_login.bind("<KeyRelease>", enable_login_button)
    entry_pass.bind("<KeyRelease>", enable_login_button)
    
    btn_login = Button(root, text="Iniciar sessão", font=("Consolas", 13), borderwidth=1, command=check_login, state=DISABLED)
    
    txt_dados.pack(pady=15)
    
    txt_login.pack(pady=5)
    entry_login.pack(pady=3)
    
    txt_pass.pack(pady=10)
    entry_pass.pack(pady=3)
    
    btn_login.pack(pady=10)

def toolbar(state="normal"):
    global robot_socket_text
    
    menu_principal = Menu(root)
    root.config(menu=menu_principal)

    item_state = "disabled" if state == "disabled" else "normal"

    menu_principal.add_command(label="História", command=new_story, font=10, state=item_state)
    menu_principal.add_command(label="Jogadores", font=10, state=item_state, command=players)
    menu_principal.add_command(label="Ajuda", font=10, state=item_state)
    menu_principal.add_command(label="Ligação", font=10, state=item_state, command=connectivity)
    
    root.imagem_green = PhotoImage(file="icons/green.png")
    root.imagem_red = PhotoImage(file="icons/red.png")
    
    if robot_socket_text is None or robot_socket_commands is None:
        menu_principal.add_command(image=root.imagem_red, state=DISABLED)
    else:
        menu_principal.add_command(image=root.imagem_green, state=DISABLED)

def get_tree(tree):
    style = ttk.Style()
    style.configure("Treeview.Heading", font=("Consolas", 12))

    tree["columns"] = ("id", "name", "age", "mmse", "mmse_text", "hobbies", "profession", "names_friends")
    tree.column("#0", width=1, anchor=N)
    tree.column("id", width=25, anchor=N)
    tree.column("name", width=150, anchor=N)
    tree.column("age", width=75, anchor=N)
    tree.column("mmse", width=50, anchor=N)
    tree.column("mmse_text", width=185, anchor=N)
    tree.column("hobbies", width=280, anchor=N)
    tree.column("profession", width=200, anchor=N)
    tree.column("names_friends", width=250, anchor=N)

    tree.heading("id", text="ID")
    tree.heading("name", text="Nome")
    tree.heading("age", text="Idade")
    tree.heading("mmse", text="MMSE")
    tree.heading("mmse_text", text="Défict Cognitivo")
    tree.heading("hobbies", text="Passatempos")
    tree.heading("profession", text="Profissão passada")
    tree.heading("names_friends", text="Familiares e Amigos")

def new_story():
    global previous_root
    previous_root = 1

    send_server("100")

    number_of_patients = int(receive_server())

    for rootantigo in root.winfo_children():  # Apagar dados anteriores
        rootantigo.destroy()

    toolbar()

    txt_hist = LabelFrame(root, text="Dados da História: ", font=("Consolas", 18), padx=5, pady=5)

    tema_frame = LabelFrame(txt_hist, text="Tema: ", font=("Consolas", 13), padx=5, pady=5)
    tema_entry = Entry(tema_frame, width=75, font=("Consolas", 13))

    topics_frame = LabelFrame(txt_hist, text="Assuntos Sensíveis: ", font=("Consolas", 13), padx=5, pady=5)
    topics_entry = Entry(topics_frame, width=75, font=("Consolas", 13))

    def validate_entries(event=None):
        if tema_entry.get() and topics_entry.get() and tree.selection():
            submeter.config(state="normal")
        else:
            submeter.config(state="disabled")

    tema_entry.bind("<KeyRelease>", validate_entries)
    topics_entry.bind("<KeyRelease>", validate_entries)
    
    def send_infos_to_story():
        send_robot("500")
        
        send_robot(tema_entry.get())
        send_robot(topics_entry.get())
        
        player_info = tree.item(tree.focus(), 'values')
        
        send_robot(player_info[1])
        send_robot(player_info[2])
        send_robot(player_info[4])
        send_robot(player_info[5])
        send_robot(player_info[6])
        send_robot(player_info[7])
        
        telling_story()

    submeter = Button(root, text="Gerar História", font=("Consolas", 16), borderwidth=1, state="disabled", command=send_infos_to_story)

    txt_player = LabelFrame(root, text="Jogadores: ", font=("Consolas", 18),)

    add_search_frame = Label(root)
    add_search_frame.place(x=975, y=233)

    root.adicionar_img = PhotoImage(file="icons/plus_1.png")
    adicionar = Button(add_search_frame, image=root.adicionar_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=new_player)

    root.search_img = PhotoImage(file="icons/search.png")
    search_label = Label(add_search_frame, image=root.search_img)

    search = Entry(add_search_frame, font=("Consolas", 11), borderwidth=2)

    def insert_default_text(event=None):
        search.delete(0, END)
        search.insert(0, "Pesquisa...")

    insert_default_text()

    search.bind("<FocusIn>", lambda event: search.delete(0, END))
    search.bind("<FocusOut>", lambda event: insert_default_text() if not search.get() else None)

    search_label.grid(row=0, column=0, padx=5)
    search.grid(row=0, column=1)
    adicionar.grid(row=0, column=2, padx=15)

    tree = ttk.Treeview(txt_player, height=15)

    data = []
    for _ in range(number_of_patients):
        id = int(receive_server())
        name = receive_server()
        age = int(receive_server())
        mmse = int(receive_server())
        mmse_text = receive_server()
        profession = receive_server()
        hobbies = receive_server()
        relations = receive_server()
        data.append((id, name, age, mmse, mmse_text, hobbies, profession, relations))

    for item in data:
        tree.insert('', 'end', values=item)

    get_tree(tree)

    def on_tree_select(event=None):
        validate_entries()

    tree.bind("<<TreeviewSelect>>", on_tree_select)

    def filter_treeview(event=None):
        search_term = search.get().lower()
        for item in tree.get_children():
            tree.delete(item)
        for item in data:
            if search_term in item[1].lower():
                tree.insert('', 'end', values=item)

    search.bind("<KeyRelease>", filter_treeview)

    txt_hist.pack(padx=5, pady=10)
    tema_frame.grid(row=2, column=1, pady=2, padx=2)
    tema_entry.grid(row=2, column=1, pady= 2, padx=5)
    topics_frame.grid(row=3, column=1, pady=2, padx=2)
    topics_entry.grid(row=3, column=1, pady=2, padx=5)

    txt_player.pack(pady=5, padx=25)
    tree.pack(padx=10, pady=25)
    submeter.pack(pady=8)

paused = False
elapsed_time = 0

def telling_story():
    for rootantigo in root.winfo_children():
        rootantigo.destroy()
    
    toolbar(state="disabled")
    
    controls_frame = Frame(root)
    controls_frame.pack(pady=40)
    
    timer_frame = Frame(root)
    timer_frame.pack()

    root.play_img = PhotoImage(file="icons/play.png")
    root.pause_img = PhotoImage(file="icons/pause.png")
    root.stop_img = PhotoImage(file="icons/stop.png")
    
    root.timer_img = PhotoImage(file="icons/clock.png")
    
    def start_story():
        global paused
        paused = False
        send_robot_commands("1")
        
        if elapsed_time == 0:
            recive_text_thread = threading.Thread(target=atualize_text)
            recive_text_thread.start()
        
        play_btn.config(state=DISABLED)
        pause_btn.config(state="normal")
        stop_btn.config(state="normal")
        
        update_timer_thread = threading.Thread(target=update_timer)
        update_timer_thread.start()

    def pause_story():
        send_robot_commands("0")

        play_btn.config(state="normal")
        pause_btn.config(state="disabled")
        stop_btn.config(state="normal")

    def stop_story():        
        response = messagebox.askyesno("Terminar História", "Tem certeza de que deseja terminar a história agora?")
        
        if response:
            play_btn.config(state="disabled")
            pause_btn.config(state="disabled")
            stop_btn.config(state="disabled")
            send_robot_commands("-1")
        
    def update_timer():
        global elapsed_time
        while True:
            if not paused:
                mins, secs = divmod(elapsed_time, 60)
                timer_label.config(text=f": {mins:02d}:{secs:02d}")
                time.sleep(1)
                elapsed_time += 1
            else:
                return
    
    def atualize_text():
        global paused
        global elapsed_time
        
        while True:
            text = receive_robot()
            if text:
                if text == "-1":
                    paused = True
                    play_btn.config(state="disabled")
                    pause_btn.config(state="disabled")
                    stop_btn.config(state="disabled")
                    messagebox.showwarning("Aviso!", "Fim da história!")
                    toolbar(state="normal")
                    elapsed_time = 0
                    break
                elif text == "YES":
                    paused = True
                elif text == "-":
                    pass
                else:
                    interactions_list.config(state=NORMAL)
                    if text.startswith("Robô"):
                        interactions_list.insert(END, f"{text}\n")
                    else:
                        interactions_list.insert(END, f"{text}\n")
                    interactions_list.config(state=DISABLED)
                    interactions_list.see(END)

    play_btn = Button(controls_frame, image=root.play_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=start_story)
    pause_btn = Button(controls_frame, image=root.pause_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=pause_story, state=DISABLED)
    stop_btn = Button(controls_frame, image=root.stop_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=stop_story, state=DISABLED)
    
    clock_label = Label(timer_frame, image=root.timer_img)
    timer_label = Label(timer_frame, text=": 00:00", font=("Consolas", 13))
    
    interactions_list = Text(root, width=115, height=18, borderwidth=0, background="gray", wrap="word", font=("Consolas", 12))
    
    play_btn.grid(row=0, column=1, padx=90)
    pause_btn.grid(row=0, column=2, padx=90)
    stop_btn.grid(row=0, column=3, padx=90)
    
    clock_label.grid(row=0, column=1)
    timer_label.grid(row=0, column=2)
    
    interactions_list.pack(pady=20)
    interactions_list.config(state=DISABLED)

def players():
    
    global previous_root
    previous_root = 2

    send_server("100")

    number_of_patients = int(receive_server())

    for rootantigo in root.winfo_children():
        rootantigo.destroy()
    
    toolbar()
    
    txt_player = LabelFrame(root, text="Lista de Jogadores: ", font=("Consolas", 18))
    txt_player.pack(pady=15, padx=25)

    add_search_frame = Label(root)
    add_search_frame.place(x=860, y=35)
    
    root.adicionar_img = PhotoImage(file="icons/plus_1.png")
    adicionar = Button(add_search_frame, image=root.adicionar_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=new_player)

    root.search_img = PhotoImage(file="icons/search.png")
    search_label = Label(add_search_frame, image=root.search_img)
    
    def edit_player():
        new_player(tree.item(tree.focus(), 'values'))

    root.edit_img = PhotoImage(file="icons/edit.png")
    edit_label = Button(add_search_frame, image=root.edit_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], state="disabled", command=edit_player)
    
    def delete_player():
        response = messagebox.askyesno("Confirmação de Exclusão", "Tem certeza de que deseja apagar este jogador?")
        
        if response:
            send_server("400")
            
            player_info = tree.item(tree.focus(), 'values')
            send_server(player_info[0])
            players()
    
    root.delete_img = PhotoImage(file="icons/trash_1.png")
    delete_label = Button(add_search_frame, image=root.delete_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], state="disabled", command=delete_player)

    def insert_default_text(event=None):
        search.delete(0, END)
        search.insert(0, "Pesquisa pelo nome")

    search = Entry(add_search_frame, font=("Consolas", 11), borderwidth=2)

    insert_default_text()

    search.bind("<FocusIn>", lambda event: search.delete(0, END))
    search.bind("<FocusOut>", lambda event: insert_default_text() if not search.get() else None)
    
    space_label = Label(add_search_frame, width=3)
    
    search_label.grid(row=0, column=0, padx=5)
    search.grid(row=0, column=1)
    space_label.grid(row=0, column=2)
    edit_label.grid(row=0, column=3, padx=10)
    adicionar.grid(row=0, column=4, padx=10)
    delete_label.grid(row=0, column=5, padx=8)

    tree = ttk.Treeview(txt_player, height=27)
    tree.pack(padx=10, pady=25)

    data = []
    for _ in range(number_of_patients):
        id = int(receive_server())
        name = receive_server()
        age = int(receive_server())
        mmse = int(receive_server())
        mmse_text = receive_server()
        profession = receive_server()
        hobbies = receive_server()
        relations = receive_server()
        data.append((id, name, age, mmse, mmse_text, hobbies, profession, relations))

    for item in data:
        tree.insert('', 'end', values=item)

    get_tree(tree)

    def on_tree_select(event=None):
        if tree.selection():
            edit_label.config(state="normal")
            delete_label.config(state="normal")

        else:
            edit_label.config(state="disabled")
            delete_label.config(state="disabled")

    tree.bind("<<TreeviewSelect>>", on_tree_select)

    def filter_treeview(event=None):
        search_term = search.get().lower()
        for item in tree.get_children():
            tree.delete(item)
        for item in data:
            if search_term in item[1].lower():
                tree.insert('', 'end', values=item)

    search.bind("<KeyRelease>", filter_treeview)

def new_player(player_info=None):
    global previous_root

    def update_scale_value():
        selection = deficit_var.get()
        if selection == 0:  # Nulo
            mmse_scale.set(30)
        elif selection == 1:  # Leve
            mmse_scale.set(26)
        elif selection == 2:  # Moderado
            mmse_scale.set(20)

    def on_closing_root():
        pass

    # Salvar o protocolo original de fechamento
    root_close_protocol = root.protocol("WM_DELETE_WINDOW", on_closing_root)
    
    def close_new_window():
        root.protocol("WM_DELETE_WINDOW", root_close_protocol)  # Restaura o protocolo de fechamento original
        new_window.destroy()
    
    # Criar a nova janela pop-up
    new_window = Toplevel(root)
    new_window.geometry("800x600")
    new_window.title("Story Telling - Novo Jogador")
    new_window.resizable(False, False)
    new_window.grab_set()  # Define um "grab" modal na new_window

    new_window.protocol("WM_DELETE_WINDOW", close_new_window)

    players_txt = LabelFrame(new_window, text="Dados do Jogador: ", font=("Consolas", 18))
    players_txt.pack(pady=2, padx=10, fill="both", expand="yes")

    name_age_deficit_frame = Frame(players_txt)
    name_age_deficit_frame.pack(pady=5)

    nome_frame = LabelFrame(name_age_deficit_frame, text="Nome: ", font=("Consolas", 13))
    nome_frame.grid(row=0, column=0, padx=15, pady=5, sticky='w')
    nome_text = Entry(nome_frame, width=25, font=("Consolas", 13))
    nome_text.pack(pady=5, padx=5)

    # Frame para a idade do jogador
    idade_frame = LabelFrame(name_age_deficit_frame, text="Idade: ", font=("Consolas", 13))
    idade_frame.grid(row=0, column=1, padx=15, pady=5, sticky='w')

    def validate_age_input(new_value):
        return new_value.isdigit() or new_value == ""

    vcmd = (new_window.register(validate_age_input), '%P')
    idade_text = Entry(idade_frame, width=5, font=("Consolas", 13), validate='key', validatecommand=vcmd)
    idade_text.pack(pady=5, padx=5)

    # Frame para os radio buttons de Déficit Cognitivo
    deficit_frame = LabelFrame(name_age_deficit_frame, text="Déficit Cognitivo: ", font=("Consolas", 13))
    deficit_frame.grid(row=0, column=2, padx=15, pady=5, sticky='w')

    # Variável para armazenar a seleção dos radio buttons
    deficit_var = IntVar()
    deficit_var.set(0)
    
    def update_scale_with_radio():
        update_scale_value()

    # Radio buttons
    radio_nulo = Radiobutton(deficit_frame, text="Nulo", variable=deficit_var, value=0, font=("Consolas", 13), command=update_scale_with_radio)
    radio_nulo.grid(row=0, column=2, sticky='w', pady=5, padx=5)
    radio_leve = Radiobutton(deficit_frame, text="Leve", variable=deficit_var, value=1, font=("Consolas", 13), command=update_scale_with_radio)
    radio_leve.grid(row=0, column=1, sticky='w', pady=5, padx=5)
    radio_moderado = Radiobutton(deficit_frame, text="Moderado", variable=deficit_var, value=2, font=("Consolas", 13), command=update_scale_with_radio)
    radio_moderado.grid(row=0, column=0, sticky='w', pady=5, padx=5)

    profession_mmse_frame = Frame(players_txt)
    profession_mmse_frame.pack(pady=5)

    # Frame para a profissão passada
    profissao_frame = LabelFrame(profession_mmse_frame, text="Profissão Passada: ", font=("Consolas", 13))
    profissao_frame.grid(row=1, column=0, padx=25, pady=5, sticky='w')
    profissao_text = Entry(profissao_frame, width=40, font=("Consolas", 13))
    profissao_text.pack(pady=5, padx=5)

    # Frame para o MMSE (Mini-Mental State Examination)
    mmse_frame = LabelFrame(profession_mmse_frame, text="MMSE: ", font=("Consolas", 13))
    mmse_frame.grid(row=1, column=1, padx=40, pady=5, sticky='w')
    
    mmse_scale = Scale(mmse_frame, from_=11, to=30, orient=HORIZONTAL, font=("Consolas", 13), length=200)
    mmse_scale.pack(pady=5, padx=5)

    update_scale_value()
    
    # Adicionando a função de atualização do Scale para o evento de mover o Scale
    def update_radio_with_scale(event):
        scale_value = mmse_scale.get()
        if scale_value >= 28:
            deficit_var.set(0)  # Nulo
        elif 21 <= scale_value <= 27:
            deficit_var.set(1)  # Leve
        elif 11 <= scale_value <= 20:
            deficit_var.set(2)  # Moderado

    mmse_scale.bind("<ButtonRelease-1>", update_radio_with_scale)

    passatempos_names_frame = Frame(players_txt)
    passatempos_names_frame.pack(pady=5)
    
    # Frame para os passatempos
    passatempos_frame = LabelFrame(passatempos_names_frame, text="Passatempos: ", font=("Consolas", 13))
    passatempos_frame.pack(side=LEFT, padx=25)

    passatempos_text = Text(passatempos_frame, width=80, height=2, font=("Consolas", 13))
    passatempos_text.pack(pady=5, padx=15)

    names_frame = LabelFrame(players_txt, text="Nomes e relações importantes: ", font=("Consolas", 13))
    names_frame.pack(pady=5, padx=5)

    tree = ttk.Treeview(names_frame, height=5)

    style = ttk.Style()
    style.configure("Treeview.Heading", font=("Consolas", 10))

    tree["columns"] = ("names", "relation")
    tree.column("#0", width=1, anchor=N)
    tree.column("names", width=400, anchor=N)
    tree.column("relation", width=300, anchor=N)

    tree.heading("names", text="Nome")
    tree.heading("relation", text="Relação")
    tree.pack(pady=15, padx=15)

    add_delete_frame = Label(new_window)
    add_delete_frame.place(x=690, y=335)
    
    def add_item(tree):
        # Função para adicionar um item à Treeview
        add_window = Toplevel(root)
        add_window.geometry("300x200")
        add_window.title("Nova Relação")
        add_window.resizable(False, False)
        add_window.grab_set()

        name_label_frame = LabelFrame(add_window, text="Nome:", font=("Consolas", 13))
        name_label_frame.pack(pady=5)
        
        name_entry = Entry(name_label_frame, width=25, font=("Consolas", 13))
        name_entry.pack(padx=5, pady=2)

        relation_label_frame = LabelFrame(add_window, text="Relação:", font=("Consolas", 13))
        relation_label_frame.pack(pady=5)
        
        relation_entry = Entry(relation_label_frame, width=25, font=("Consolas", 13))
        relation_entry.pack(padx=5, pady=2)

        def confirm_add():
            name = name_entry.get().strip()
            relation = relation_entry.get().strip()
            if name and relation:
                tree.insert('', 'end', values=(name, relation))
                add_window.destroy()
                check_fields_filled()  # Verificar os campos novamente após adicionar um item

        confirm_btn = Button(add_window, text="Adicionar", font=("Consolas", 13), command=confirm_add)
        confirm_btn.pack(pady=10)

        add_window.grab_set()
        root.wait_window(add_window)
    
    def delete_item(tree):
        selected_item = tree.selection()
        if selected_item:
            tree.delete(selected_item)
            check_fields_filled()  # Verificar os campos novamente após excluir um item

    def update_delete_button_state(event):
        if tree.selection():
            delete_label.config(state=NORMAL)
        else:
            delete_label.config(state=DISABLED)
    
    tree.bind('<<TreeviewSelect>>', update_delete_button_state)

    new_window.plus_img = PhotoImage(file="icons/plus_2.png")
    plus_label = Button(add_delete_frame, image=new_window.plus_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], command=lambda: add_item(tree))

    new_window.delete_img = PhotoImage(file="icons/trash.png")
    delete_label = Button(add_delete_frame, image=new_window.delete_img, borderwidth=0, activebackground=root['bg'], activeforeground=root['bg'], state=DISABLED, command=lambda: delete_item(tree))

    plus_label.grid(row=0, column=0, padx=10)
    delete_label.grid(row=0, column=1)

    btns_frame = Frame(new_window)
    btns_frame.pack(pady=13)

    def check_fields_filled():
        # Verifica se todos os campos estão preenchidos e se há elementos na tree
        if (nome_text.get().strip() and idade_text.get().strip() and profissao_text.get().strip() and 
            passatempos_text.get("1.0", "end-1c").strip() and tree.get_children()):
            save_btn.config(state=NORMAL)
        else:
            save_btn.config(state=DISABLED)

    nome_text.bind("<KeyRelease>", lambda event: check_fields_filled())
    idade_text.bind("<KeyRelease>", lambda event: check_fields_filled())
    profissao_text.bind("<KeyRelease>", lambda event: check_fields_filled())
    passatempos_text.bind("<KeyRelease>", lambda event: check_fields_filled())

    def save_player():
        if player_info:
            send_server("300")
            send_server(player_info[0])
        else:
            send_server("200")

        send_server(nome_text.get().strip())
        send_server(idade_text.get().strip())
        send_server(profissao_text.get().strip())
        send_server(str(passatempos_text.get("1.0", "end-1c").strip()))
        send_server(str(mmse_scale.get()))
        
        deficit_mapping = {0: "Nulo", 1: "Leve", 2: "Moderado"}
        send_server(deficit_mapping[deficit_var.get()])

        nomes_relacoes = []
        for item in tree.get_children():
            nome_relacao = tree.item(item, "values")
            nomes_relacoes.append(f"{str(nome_relacao[0]).strip()}:{str(nome_relacao[1]).strip()}")
        
        # Converter nomes_relacoes para uma string separada por pontos e vírgulas
        nomes_relacoes_str = "; ".join(nomes_relacoes)
        
        # Enviar a string para o servidor
        send_server(nomes_relacoes_str)

        data = receive_server()

        if data == "0":
            messagebox.showwarning("Aviso!", "Dados guardados com sucesso!")
            new_window.destroy()

            if(previous_root == 1):
                new_story()
            else:
                players()
        else:
            messagebox.showerror("Erro!", "Impossível guardar dados!")

    save_btn = Button(btns_frame, text="Guardar", font=("Consolas", 13), borderwidth=1, state=DISABLED, width=15, command=save_player)

    def cancel_action():
        if (nome_text.get() or idade_text.get() or profissao_text.get() or 
            passatempos_text.get("1.0", "end-1c") or tree.get_children()):
            if player_info:
                if messagebox.askokcancel("Cancelar", "Tem a certeza que deseja cancelar? Os dados alterados não serão guardados."):
                    new_window.destroy()
            else:
                if messagebox.askokcancel("Cancelar", "Tem a certeza que deseja cancelar? Os dados introduzidos não serão guardados."):
                    new_window.destroy()
        else:
            new_window.destroy()

    cancel_btn = Button(btns_frame, text="Cancelar", font=("Consolas", 13), borderwidth=1, width=15, command=cancel_action)

    if player_info:
        nome_text.insert(0, player_info[1])
        idade_text.insert(0, player_info[2])
        profissao_text.insert(0, player_info[6])
        passatempos_text.insert(1.0, player_info[5])
        mmse_scale.set(player_info[3])

        for name_relation in player_info[7].split(';'):
            name, relation = name_relation.split(':')
            tree.insert('', 'end', values=(name.strip(), relation.strip()))
        
        update_radio_with_scale(None)
        save_btn.config(state=NORMAL)

    save_btn.grid(row=0, column=0, padx=20)
    cancel_btn.grid(row=0, column=1, padx=20)

    new_window.grab_set()
    root.wait_window(new_window)

def connectivity():
    
    global HOST_ROBOT
    global robot_socket_commands
    global robot_socket_text
    
    for widget in root.winfo_children():
        widget.destroy()

    toolbar()

    root.imagem_green_1 = PhotoImage(file="icons/green.png")
    root.imagem_red_1 = PhotoImage(file="icons/red.png")
    root.imagem_edit = PhotoImage(file="icons/edit.png")
    root.imagem_refresh = PhotoImage(file="icons/refresh.png")

    main_frame = LabelFrame(root, text="Conexões: ", font=("Consolas", 20), padx=20, pady=20)
    main_frame.pack(pady=25, padx=25, fill="both", expand=True)

    server_frame = LabelFrame(main_frame, text="Servidor: ", font=("Consolas", 18), padx=20, pady=20)
    server_frame.pack(pady=10, fill="x")

    ip_frame_server = Label(server_frame, font=("Consolas", 15), text="Domínio: ISR UC", anchor='w')
    ip_frame_server.grid(row=0, column=0, sticky='w', pady=25, padx=15)
    status_frame_server = Label(server_frame, font=("Consolas", 15), text="Estado: Conectado", anchor='w')
    status_frame_server.grid(row=1, column=0, sticky='w', pady=25, padx=15)    
    image_frame_server = Label(server_frame, image=root.imagem_green_1)
    image_frame_server.grid(row=1, column=1, pady=5, padx=5)
    nota_frame_server = Label(root, font=("Consolas", 10), text="Nota: Ligação estabelecida com o servidor do ISR da Universidade de Coimbra que funciona como banco de dados de todos os pacientes!", anchor='w')
    nota_frame_server.place(x=85, y=286)

    robot_frame = LabelFrame(main_frame, text="Robô: ", font=("Consolas", 18), padx=20, pady=20)
    robot_frame.pack(pady=30, fill="x")

    robot_ip_text = StringVar()
    robot_ip_text.set("Endereço IP: " + HOST_ROBOT)
    ip_frame_robot = Label(robot_frame, font=("Consolas", 15), textvariable=robot_ip_text, anchor='w')
    ip_frame_robot.grid(row=0, column=0, sticky='w', pady=25, padx=15)
    status_frame_robot = Label(robot_frame, font=("Consolas", 15), text="Estado: Conectado", anchor='w')
    status_frame_robot.grid(row=1, column=0, sticky='w', pady=25, padx=15)
    
    if robot_socket_text is None or robot_socket_commands is None:
        status_frame_robot.config(text="Estado: Desconectado")
        image_frame_robot = Label(robot_frame, image=root.imagem_red_1)
    
    else:
        image_frame_robot = Label(robot_frame, image=root.imagem_green_1)
        
    image_frame_robot.grid(row=1, column=1, pady=5, padx=5)
    nota_frame_robot = Label(root, font=("Consolas", 10), text="Nota: Ligação entre este software e o robô! Possível necessidade de alteração de endereço de IP!", anchor='w')
    nota_frame_robot.place(x=85, y=555)

    def edit_ip(ip_variable):        
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
            
            send_server("600")
            
            if robot_socket_text or robot_socket_commands is not None:
                send_robot("CLOSE")
                robot_socket_text.close()
                robot_socket_commands.close()

            new_ip = ip_entry_var.get()
            global HOST_ROBOT
            HOST_ROBOT = new_ip
            
            send_server(HOST_ROBOT)
            
            ip_variable.set("Endereço IP: " + HOST_ROBOT)

            connection_robot_text()
            time.sleep(0.5)
            connection_robot_commands()
            
            if robot_socket_text is None or robot_socket_commands is None:
                messagebox.showwarning("Aviso!", "IP sem conexão")

            dialog.destroy()
            connectivity()

        dialog = Toplevel(root)
        dialog.title("Novo IP")
        dialog.geometry("300x150")
        dialog.transient(root)
        dialog.grab_set()

        Label(dialog, text="Insira o novo IP do robô:", font=("Consolas", 13)).pack(pady=10)
        
        ip_entry_var = StringVar()
        ip_entry_var.trace("w", validate_ip)
        
        ip_entry = Entry(dialog, textvariable=ip_entry_var, font=("Consolas", 13))
        ip_entry.pack(pady=5)
        
        confirm_button = Button(dialog, text="Confirmar", command=on_confirm, font=("Consolas", 13), state='disabled')
        confirm_button.pack(pady=10)
        
    edit_button = Button(robot_frame, image=root.imagem_edit, command=lambda ip_var=robot_ip_text: edit_ip(ip_var), font=("Consolas", 12), borderwidth=0)
    edit_button.grid(row=0, column=1, columnspan=2, pady=5, padx=5)
    
    def refresh_connection():
        
        global robot_socket_commands
        global robot_socket_text
        
        if robot_socket_text is None or robot_socket_commands is None:
            connection_robot_text()
            time.sleep(0.5)
            connection_robot_commands()
            
            if robot_socket_text is None or robot_socket_commands is None:
                messagebox.showwarning("Aviso!", "IP sem conexão")
            
            connectivity()
        else:
            pass
    
    refresh_button = Button(robot_frame, image=root.imagem_refresh, command=refresh_connection, font=("Consolas", 12), borderwidth=0)
    refresh_button.grid(row=0, column=2, columnspan=2, pady=5, padx=50)
    

    Label(main_frame, text="", font=("Consolas", 1)).pack(pady=10)
    
def main():
    
    global server_socket
    
    connection_server()
    
    if server_socket is None:
        return
            
    connection_robot_text()
    time.sleep(0.5)
    connection_robot_commands()
    
    login()
    
    root.mainloop()

if __name__ == "__main__":
    root = Tk()
    root.title("Story Telling - EuroAge+")
    root.geometry('500x300')
    root.resizable(False, False)
    main()
