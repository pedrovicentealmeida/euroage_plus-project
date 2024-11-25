from tkinter import Label, LabelFrame, Entry, Button, PhotoImage, StringVar

from gui_components.CustomTreeView import CustomTreeView

from gui.new_player_window import NewPlayerWindow
from gui.telling_story_window import TellingStoryWindow

from utils.helpers import PatientDataFetcher

class NewStoryWindow:
    def __init__(self, root, server, robot, toolbar):
        self.root = root
        self.toolbar = toolbar
        self.server = server
        self.robot = robot
        self.create_gui_elements()

    def create_gui_elements(self):
        self.data = PatientDataFetcher(self.server).fetch_patient_data()
        self.create_story_frame()
        self.create_players_frame()
        self.create_submit_button()

    def create_story_frame(self):
        story_frame = LabelFrame(self.root, text="Dados da História: ", font=("Consolas", 18), padx=5, pady=5)
        story_frame.pack(padx=5, pady=10)

        self.create_theme_frame(story_frame)
        self.create_topics_frame(story_frame)

    def create_theme_frame(self, parent):
        theme_frame = LabelFrame(parent, text="Tema: ", font=("Consolas", 13), padx=5, pady=5)
        theme_frame.grid(row=2, column=1, pady=2, padx=2)

        self.theme_entry = Entry(theme_frame, width=75, font=("Consolas", 13))
        self.theme_entry.grid(row=2, column=1, pady=2, padx=5)
        self.theme_entry.bind("<KeyRelease>", self.validate_entries)

    def create_topics_frame(self, parent):
        topics_frame = LabelFrame(parent, text="Assuntos Sensíveis: ", font=("Consolas", 13), padx=5, pady=5)
        topics_frame.grid(row=3, column=1, pady=2, padx=2)

        self.topics_entry = Entry(topics_frame, width=75, font=("Consolas", 13))
        self.topics_entry.grid(row=3, column=1, pady=2, padx=5)
        self.topics_entry.bind("<KeyRelease>", self.validate_entries)

    def create_players_frame(self):
        players_frame = LabelFrame(self.root, text="Jogadores: ", font=("Consolas", 18), padx=15, pady=40)
        players_frame.pack(pady=5, padx=25)

        self.create_search_add_frame()
        self.create_treeview(players_frame)

    def create_search_add_frame(self):
        frame = Label(self.root)
        frame.place(x=975, y=233)

        self.create_add_button(frame)
        self.create_search_entry(frame)

    def create_add_button(self, parent):
        self.root.adicionar_img = PhotoImage(file="icons/plus_1.png")
        adicionar = Button(parent, image=self.root.adicionar_img, borderwidth=0, 
                           activebackground=self.root['bg'], activeforeground=self.root['bg'], 
                           command=lambda: NewPlayerWindow(self.root, self.server, None))
        adicionar.grid(row=0, column=2, padx=15)

    def create_search_entry(self, parent):
        self.root.search_img = PhotoImage(file="icons/search.png")
        search_label = Label(parent, image=self.root.search_img)
        search_label.grid(row=0, column=0, padx=5)

        self.search_var = StringVar()
        self.search_var.set("Pesquisa...")
        self.search = Entry(parent, font=("Consolas", 11), borderwidth=2, textvariable=self.search_var)
        self.search.grid(row=0, column=1)

        self.search.bind("<FocusIn>", self.clear_search_text)
        self.search.bind("<FocusOut>", self.restore_search_text)
        self.search.bind("<KeyRelease>", self.filter_treeview)

    def create_treeview(self, parent):
        self.tree_view = CustomTreeView(parent, height=None)
        self.tree_view.insert_data(self.data)
        self.tree_view.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

    def create_submit_button(self):
        self.submit_button = Button(self.root, text="Gerar História", font=("Consolas", 16), 
                                    borderwidth=1, state="disabled", command=self.send_infos_to_story)
        self.submit_button.pack(pady=8)

    def validate_entries(self, event=None):
        if self.theme_entry.get() and self.topics_entry.get() and self.tree_view.tree.selection():
            self.submit_button.config(state="normal")
        else:
            self.submit_button.config(state="disabled")

    def send_infos_to_story(self):
        self.robot.send_commands_robot("500")
        
        self.robot.send_commands_robot(self.theme_entry.get())
        self.robot.send_commands_robot(self.topics_entry.get())
        
        selected_item = self.tree_view.tree.selection()
        
        player_info = self.tree_view.tree.item(selected_item[0], 'values')
        self.robot.send_commands_robot(player_info[1])
        self.robot.send_commands_robot(player_info[2])
        self.robot.send_commands_robot(player_info[4])
        self.robot.send_commands_robot(player_info[5])
        self.robot.send_commands_robot(player_info[6])
        self.robot.send_commands_robot(player_info[7])

        TellingStoryWindow(self.root, self.server, self.robot, self.toolbar)

    def on_tree_select(self, event=None):
        self.validate_entries()

    def filter_treeview(self, event=None):
        search_term = self.search_var.get().lower()
        if search_term == "pesquisa...":
            search_term = ""
        self.tree_view.tree.delete(*self.tree_view.tree.get_children())
        for item in self.data:
            if search_term in item[1].lower():
                self.tree_view.tree.insert('', 'end', values=item)

    def clear_search_text(self, event):
        if self.search_var.get() == "Pesquisa...":
            self.search_var.set("")

    def restore_search_text(self, event):
        if not self.search_var.get():
            self.search_var.set("Pesquisa...")