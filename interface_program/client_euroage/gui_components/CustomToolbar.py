from utils.helpers import clear_root
from tkinter import Menu, PhotoImage

class CustomToolbar:
    def __init__(self, client):
        self.client = client
        self.menu_principal = None
        self.item_state = "normal"
        self.status_icon_item = None

    def change_ip(self, robot):
        self.robot = robot

    def display(self, root, robot_socket):
        self.root = root
        self.robot = robot_socket
        
        if self.menu_principal is None:
            self.menu_principal = Menu(self.root)
            self.root.config(menu=self.menu_principal)

        self.item_state = "normal"
        self.add_menu_items()
        self.load_status_icon()

    def add_menu_items(self):
        self.menu_principal.add_command(label="História", font=10, state=self.item_state, command=self.new_story)
        self.menu_principal.add_command(label="Jogadores", font=10, state=self.item_state, command=self.players)
        self.menu_principal.add_command(label="Ajuda", font=10, state=self.item_state)
        self.menu_principal.add_command(label="Ligação", font=10, state=self.item_state, command=self.connectivity)

    def load_status_icon(self):
        try:
            self.root.imagem_green = PhotoImage(file="icons/green.png")
            self.root.imagem_red = PhotoImage(file="icons/red.png")
        except Exception as e:
            print(f"Error loading images: {e}")
            return
        
        if self.robot is None or self.robot is None:
            self.update_status_icon(True)
        else:
            self.update_status_icon(False)

    def update_status_icon(self, connected):
        self.menu_principal.delete(0, "end")
        self.add_menu_items()

        icon = self.root.imagem_green if connected else self.root.imagem_red
        self.status_icon_item = self.menu_principal.add_command(image=icon, state="disabled")

    def new_story(self):
        from gui.new_story_window import NewStoryWindow
        clear_root(self.root)
        NewStoryWindow(self.root, self.client, self.robot, self)

    def players(self):
        from gui.players_window import PlayersWindow
        clear_root(self.root)
        PlayersWindow(self.root, self.client)

    def connectivity(self):
        from gui.connectivity_window import ConnectivityWindow
        clear_root(self.root)
        ConnectivityWindow(self.root, self.client, self.robot, self)

    def toggle_state(self, state):
        self.item_state = "disabled" if state == "disabled" else "normal"
        
        self.menu_principal.delete(0, "end")
        self.add_menu_items()
        self.load_status_icon()
