from utils.helpers import clear_root
from tkinter import Menu, PhotoImage

class CustomToolbar:
    def __init__(self, client, robot_socket_text=None, robot_socket_commands=None):
        self.client = client
        self.robot_socket_text = robot_socket_text
        self.robot_socket_commands = robot_socket_commands
        self.menu_principal = None

    def display(self, root, state="normal"):
        self.root = root
        self.state = state
        
        if self.menu_principal is None:
            self.menu_principal = Menu(self.root)
            self.root.config(menu=self.menu_principal)

        self.item_state = "disabled" if state == "disabled" else "normal"
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

        if self.robot_socket_text is None or self.robot_socket_commands is None:
            self.menu_principal.add_command(image=self.root.imagem_red, state="disabled")
        else:
            self.menu_principal.add_command(image=self.root.imagem_green, state="disabled")
    
    def new_story(self):
        from gui.new_story_window import NewStoryWindow
        clear_root(self.root)
        NewStoryWindow(self.root, self.client)

    def players(self):
        from gui.players_window import PlayersWindow
        clear_root(self.root)
        PlayersWindow(self.root, self.client)

    def connectivity(self):
        print("Connectivity selected")