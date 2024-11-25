from tkinter import Toplevel, Frame, Label, LabelFrame, Entry, Button, NORMAL, DISABLED, N, LEFT, HORIZONTAL, Scale, PhotoImage, IntVar, Radiobutton, Text, ttk, messagebox
from typing import Optional, List, Tuple
from network.client import SocketClientServer

class NewPlayerWindow:
    def __init__(self, root, client: SocketClientServer, player_info: Optional[List] = None):
        self.root = root
        self.client = client
        self.player_info = player_info
        self.previous_root = 1

        self.root_close_protocol = self.root.protocol("WM_DELETE_WINDOW", self.on_closing_root)

        self.new_window = self.create_new_window()
        self.create_gui_elements()

        self.new_window.grab_set()
        root.wait_window(self.new_window)

    def create_new_window(self) -> Toplevel:
        new_window = Toplevel(self.root)
        new_window.geometry("800x600")
        new_window.title("Story Telling - Novo Jogador")
        new_window.resizable(False, False)
        new_window.protocol("WM_DELETE_WINDOW", self.close_new_window)
        return new_window

    def create_gui_elements(self):
        players_txt = self.create_player_frame()
        self.create_name_age_deficit_frame(players_txt)
        self.create_profession_mmse_frame(players_txt)
        self.create_hobbies_frame(players_txt)
        self.create_names_relations_frame(players_txt)
        self.create_buttons_frame()

        self.load_player_info()
        self.bind_events()

    def create_player_frame(self) -> LabelFrame:
        players_txt = LabelFrame(self.new_window, text="Dados do Jogador: ", font=("Consolas", 18))
        players_txt.pack(pady=2, padx=10, fill="both", expand="yes")
        return players_txt

    def create_name_age_deficit_frame(self, parent: LabelFrame):
        frame = Frame(parent)
        frame.pack(pady=5)

        self.nome_text = self.create_labeled_entry(frame, "Nome: ", 25, 0, 0)
        vcmd = (self.new_window.register(self.validate_age_input), '%P')
        self.idade_text = self.create_labeled_entry(frame, "Idade: ", 5, 0, 1, validate_cmd=vcmd)
        self.create_deficit_frame(frame)

    def create_profession_mmse_frame(self, parent: LabelFrame):
        frame = Frame(parent)
        frame.pack(pady=5)

        self.profissao_text = self.create_labeled_entry(frame, "Profissão Passada: ", 40, 1, 0)
        self.create_mmse_scale(frame)

    def create_hobbies_frame(self, parent: LabelFrame):
        frame = Frame(parent)
        frame.pack(pady=5)

        passatempos_frame = LabelFrame(frame, text="Passatempos: ", font=("Consolas", 13))
        passatempos_frame.pack(side=LEFT, padx=25)

        self.passatempos_text = Text(passatempos_frame, width=80, height=2, font=("Consolas", 13))
        self.passatempos_text.pack(pady=5, padx=15)

    def create_names_relations_frame(self, parent: LabelFrame):
        frame = LabelFrame(parent, text="Nomes e relações importantes: ", font=("Consolas", 13))
        frame.pack(pady=5, padx=5)

        self.create_treeview(frame)
        self.create_add_delete_buttons()

    def create_treeview(self, parent: LabelFrame):
        self.tree = ttk.Treeview(parent, height=5)
        style = ttk.Style()
        style.configure("Treeview.Heading", font=("Consolas", 10))

        self.tree["columns"] = ("names", "relation")
        self.tree.column("#0", width=1, anchor=N)
        self.tree.column("names", width=400, anchor=N)
        self.tree.column("relation", width=300, anchor=N)

        self.tree.heading("names", text="Nome")
        self.tree.heading("relation", text="Relação")
        self.tree.pack(pady=15, padx=15)

    def create_add_delete_buttons(self):
        frame = Label(self.new_window)
        frame.place(x=690, y=335)

        self.new_window.plus_img = PhotoImage(file="icons/plus_2.png")
        plus_label = Button(frame, image=self.new_window.plus_img, borderwidth=0, 
                            activebackground=self.root['bg'], activeforeground=self.root['bg'], 
                            command=lambda: self.add_item(self.tree))

        self.new_window.delete_img = PhotoImage(file="icons/trash.png")
        self.delete_label = Button(frame, image=self.new_window.delete_img, borderwidth=0, 
                                   activebackground=self.root['bg'], activeforeground=self.root['bg'], 
                                   state=DISABLED, command=lambda: self.delete_item(self.tree))

        plus_label.grid(row=0, column=0, padx=10)
        self.delete_label.grid(row=0, column=1)

    def create_buttons_frame(self):
        frame = Frame(self.new_window)
        frame.pack(pady=13)

        self.save_btn = Button(frame, text="Guardar", font=("Consolas", 13), borderwidth=1, 
                               state=DISABLED, width=15, command=self.save_player)
        cancel_btn = Button(frame, text="Cancelar", font=("Consolas", 13), borderwidth=1, 
                            width=15, command=self.cancel_action)

        self.save_btn.grid(row=0, column=0, padx=20)
        cancel_btn.grid(row=0, column=1, padx=20)

    def create_labeled_entry(self, parent: Frame, label: str, width: int, row: int, column: int, 
                             validate_cmd: Optional[Tuple] = None) -> Entry:
        frame = LabelFrame(parent, text=label, font=("Consolas", 13))
        frame.grid(row=row, column=column, padx=15, pady=5, sticky='w')

        entry = Entry(frame, width=width, font=("Consolas", 13))
        if validate_cmd:
            entry.config(validate='key', validatecommand=validate_cmd)
        entry.pack(pady=5, padx=5)
        return entry

    def create_deficit_frame(self, parent: Frame):
        frame = LabelFrame(parent, text="Déficit Cognitivo: ", font=("Consolas", 13))
        frame.grid(row=0, column=2, padx=15, pady=5, sticky='w')

        self.deficit_var = IntVar()
        self.deficit_var.set(0)

        deficit_options = [("Nulo", 0), ("Leve", 1), ("Moderado", 2)]
        for text, value in deficit_options:
            Radiobutton(frame, text=text, variable=self.deficit_var, value=value, 
                        font=("Consolas", 13), command= self.update_scale_with_radio).grid(row=0, column=value, sticky='w', pady=5, padx=5)

    def create_mmse_scale(self, parent: Frame):
        mmse_frame = LabelFrame(parent, text="MMSE: ", font=("Consolas", 13))
        mmse_frame.grid(row=1, column=1, padx=40, pady=5, sticky='w')

        self.mmse_scale = Scale(mmse_frame, from_=11, to=30, orient=HORIZONTAL, font=("Consolas", 13), length=200)
        self.mmse_scale.pack(pady=5, padx=5)
        self.update_scale_value()
        self.mmse_scale.bind("<ButtonRelease-1>", self.update_radio_with_scale)

    def load_player_info(self):
        if self.player_info:
            self.nome_text.insert(0, self.player_info[1])
            self.idade_text.insert(0, self.player_info[2])
            self.profissao_text.insert(0, self.player_info[6])
            self.passatempos_text.insert(1.0, self.player_info[5])
            self.mmse_scale.set(self.player_info[3])

            for name_relation in self.player_info[7].split(';'):
                name, relation = name_relation.split(':')
                self.tree.insert('', 'end', values=(name.strip(), relation.strip()))

            self.update_radio_with_scale(None)
            self.save_btn.config(state=NORMAL)

    def bind_events(self):
        self.nome_text.bind("<KeyRelease>", lambda event: self.check_fields_filled())
        self.idade_text.bind("<KeyRelease>", lambda event: self.check_fields_filled())
        self.profissao_text.bind("<KeyRelease>", lambda event: self.check_fields_filled())
        self.passatempos_text.bind("<KeyRelease>", lambda event: self.check_fields_filled())
        self.tree.bind('<<TreeviewSelect>>', self.update_delete_button_state)

    def on_closing_root(self):
        pass

    def close_new_window(self):
        self.root.protocol("WM_DELETE_WINDOW", self.root_close_protocol)
        self.new_window.destroy()

    def validate_age_input(self, new_value: str) -> bool:
        return new_value.isdigit() or new_value == ""

    def update_scale_value(self):
        selection = self.deficit_var.get()
        if selection == 0:  # Nulo
            self.mmse_scale.set(30)
        elif selection == 1:  # Leve
            self.mmse_scale.set(26)
        elif selection == 2:  # Moderado
            self.mmse_scale.set(20)

    def update_scale_with_radio(self):
        self.update_scale_value()

    def update_radio_with_scale(self, event):
        scale_value = self.mmse_scale.get()
        if scale_value >= 28:
            self.deficit_var.set(0)  # Nulo
        elif 21 <= scale_value <= 27:
            self.deficit_var.set(1)  # Leve
        elif 11 <= scale_value <= 20:
            self.deficit_var.set(2)  # Moderado

    def add_item(self, tree):
        add_window = Toplevel(self.root)
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

        confirm_btn = Button(add_window, text="Adicionar", font=("Consolas", 13), 
                             command=lambda: self.confirm_add(name_entry, relation_entry, tree, add_window))
        confirm_btn.pack(pady=10)

        add_window.grab_set()
        self.root.wait_window(add_window)

    def confirm_add(self, name_entry: Entry, relation_entry: Entry, tree: ttk.Treeview, add_window: Toplevel):
        name = name_entry.get().strip()
        relation = relation_entry.get().strip()
        if name and relation:
            tree.insert('', 'end', values=(name, relation))
            add_window.destroy()
            self.check_fields_filled()

    def delete_item(self, tree):
        selected_item = tree .selection()
        if selected_item:
            tree.delete(selected_item)
            self.check_fields_filled()

    def update_delete_button_state(self, event):
        self.delete_label.config(state=NORMAL if self.tree.selection() else DISABLED)

    def check_fields_filled(self):
        if (self.nome_text.get().strip() and self.idade_text.get().strip() and 
            self.profissao_text.get().strip() and self.passatempos_text.get("1.0", "end-1c").strip() and 
            self.tree.get_children()):
            self.save_btn.config(state=NORMAL)
        else:
            self.save_btn.config(state=DISABLED)

    def save_player(self):
        if self.player_info:
            self.client.send_message("300")
            self.client.send_message(self.player_info[0])
        else:
            self.client.send_message("200")

        self.client.send_message(self.nome_text.get().strip())
        self.client.send_message(self.idade_text.get().strip())
        self.client.send_message(self.profissao_text.get().strip())
        self.client.send_message(str(self.passatempos_text.get("1.0", "end-1c").strip()))
        self.client.send_message(str(self.mmse_scale.get()))

        deficit_mapping = {0: "Nulo", 1: "Leve", 2: "Moderado"}
        self.client.send_message(deficit_mapping[self.deficit_var.get()])

        nomes_relacoes = [f"{self.tree.item(item, 'values')[0].strip()}:{self.tree.item(item, 'values')[1].strip()}" 
                          for item in self.tree.get_children()]
        
        nomes_relacoes_str = "; ".join(nomes_relacoes)
        self.client.send_message(nomes_relacoes_str)

        data = self.client.receive_message()

        if data == "0":
            messagebox.showwarning("Aviso!", "Dados guardados com sucesso!")
            self.new_window.destroy()
            print("Vamos para uma nova história!" if self.previous_root == 1 else "Vamos para os players!")
        else:
            messagebox.showerror("Erro!", "Impossível guardar dados!")

    def cancel_action(self):
        if (self.nome_text.get() or self.idade_text.get() or self.profissao_text.get() or 
            self.passatempos_text.get("1.0", "end-1c") or self.tree.get_children()):
            if self.player_info:
                if messagebox.askokcancel("Cancelar", "Tem a certeza que deseja cancelar? Os dados alterados não serão guardados."):
                    self.new_window.destroy()
            else:
                if messagebox.askokcancel("Cancelar", "Tem a certeza que deseja cancelar? Os dados introduzidos não serão guardados."):
                    self.new_window.destroy()
        else:
            self.new_window.destroy()