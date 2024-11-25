from tkinter import LabelFrame, Label, PhotoImage, Button, messagebox, Entry, END
from gui_components.CustomTreeView import CustomTreeView
from network.client import SocketClientServer
from utils.helpers import PatientDataFetcher

class PlayersWindow:
    def __init__(self, root, client: SocketClientServer):
        self.root = root
        self.client = client
        self.create_gui_elements()

    def create_gui_elements(self):
        self.data = PatientDataFetcher(self.client).fetch_patient_data()
        #CustomToolbar(self.root, self.client, "normal")
        self.create_player_label()
        self.create_search_frame()

    def create_player_label(self):
        txt_player = LabelFrame(self.root, text="Lista de Jogadores: ", font=("Consolas", 18))
        txt_player.pack(pady=15, padx=25)
        self.create_treeview(txt_player)

    def create_search_frame(self):
        search_frame = Label(self.root)
        search_frame.place(x=860, y=35)
        self.create_search_components(search_frame)  # Fixed typo in method name

    def create_search_components(self, parent):
        try:
            self.root.adicionar_img = PhotoImage(file="icons/plus_1.png")
            self.root.search_img = PhotoImage(file="icons/search.png")
            self.root.edit_img = PhotoImage(file="icons/edit.png")
            self.root.delete_img = PhotoImage(file="icons/trash_1.png")

            adicionar = Button(parent, 
                             image=self.root.adicionar_img, 
                             borderwidth=0, 
                             activebackground=parent['bg'], 
                             activeforeground=parent['bg'])
            
            search_label = Label(parent, image=self.root.search_img)
            
            self.edit_label = Button(parent, 
                                   image=self.root.edit_img, 
                                   borderwidth=0, 
                                   activebackground=parent['bg'], 
                                   activeforeground=parent['bg'], 
                                   state="disabled", 
                                   command=self.edit_player)
            
            self.delete_label = Button(parent, 
                                     image=self.root.delete_img, 
                                     borderwidth=0, 
                                     activebackground=parent['bg'], 
                                     activeforeground=parent['bg'], 
                                     state="disabled", 
                                     command=self.delete_player)
            
            self.search = Entry(parent, font=("Consolas", 11), borderwidth=2)
            space_label = Label(parent, width=3)

            self.insert_default_text()

            self.search.bind("<FocusIn>", lambda event: self.search.delete(0, END))
            self.search.bind("<FocusOut>", lambda event: self.insert_default_text() if not self.search.get() else None)
            self.search.bind("<KeyRelease>", self.filter_treeview)

            search_label.grid(row=0, column=0, padx=5)
            self.search.grid(row=0, column=1)
            space_label.grid(row=0, column=2)
            self.edit_label.grid(row=0, column=3, padx=10)
            adicionar.grid(row=0, column=4, padx=10)
            self.delete_label.grid(row=0, column=5, padx=8)
        except Exception as e:
            print(f"Error creating search components: {e}")

    def edit_player(self):
        print("YUPIII")

    def delete_player(self):
        response = messagebox.askyesno("Confirmação de Exclusão", 
                                     "Tem certeza de que deseja apagar este jogador?")
        if response:
            print("APAGOU TUDO")

    def insert_default_text(self):
        self.search.delete(0, END)
        self.search.insert(0, "Pesquisa pelo nome")

    def create_treeview(self, parent):
        self.tree_view = CustomTreeView(parent, height=23)
        self.tree_view.insert_data(self.data)
        self.tree_view.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

    def on_tree_select(self, event=None):
        if self.tree_view.tree.selection():  # Added .tree to access the actual Treeview widget
            self.edit_label.config(state="normal")
            self.delete_label.config(state="normal")
        else:
            self.edit_label.config(state="disabled")
            self.delete_label.config(state="disabled")

    def filter_treeview(self, event=None):
        search_term = self.search.get().lower()
        self.tree_view.tree.delete(*self.tree_view.tree.get_children())  # Fixed treeview clearing
        for item in self.data:
            if search_term in item[1].lower():
                self.tree_view.tree.insert('', 'end', values=item)