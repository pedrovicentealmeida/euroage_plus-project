from gui.login_window import LoginWindow
from gui.new_story_window import NewStoryWindow

from network.client import SocketClient

from gui_components.CustomToolbar import CustomToolbar

from utils.helpers import clear_root

import tkinter as tk

class AppController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Story Telling - EuroAge+")
        self.root.resizable(False, False)

        self.client = SocketClient()
        self.toolbar = CustomToolbar(self.client) 

        self.show_login_window()
        self.root.mainloop()

    def show_login_window(self):
        LoginWindow(self.root, self.client, on_login_success=self.show_new_story_window)

    def show_new_story_window(self):
        clear_root(self.root)
        self.root.geometry('1300x700')
        NewStoryWindow(self.root, self.client)
        self.toolbar.display(self.root) 

def main():    
    AppController()

if __name__ == "__main__":
    main()