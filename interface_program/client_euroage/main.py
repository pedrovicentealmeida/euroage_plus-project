import tkinter as tk
from tkinter import messagebox

from gui.login_window import LoginWindow
from gui.new_story_window import NewStoryWindow
from gui.connectivity_window import ConnectivityWindow

from network.client import SocketClientServer, SocketClientRobot

from gui_components.CustomToolbar import CustomToolbar

from utils.helpers import clear_root

import config

class AppController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry('500x300')
        self.root.title("Story Telling - EuroAge+")
        self.root.resizable(False, False)
        
        self.server = SocketClientServer()
        self.robot = None
        self.toolbar = CustomToolbar(self.server)
        
    def login_window(self):
        
        if self.server.connect() == True:
            login = LoginWindow(self.root, self.server)

            while not login.login_check:
                self.root.update()
            
            config.ROBOT_HOST =  self.server.receive_message_server()
            self.robot = SocketClientRobot(config.ROBOT_HOST)
            
            if not self.robot.connect(config.ROBOT_HOST):
                clear_root(self.root)
                self.toolbar.display(self.root, self.robot)
                self.root.geometry('1300x700')
                
                ConnectivityWindow(self.root, self.server, self.robot, self.toolbar)
                messagebox.showwarning("Aviso!", "Robô não encontrado!")
            
            else:
                clear_root(self.root)
                self.toolbar.display(self.root, self.robot)
                self.toolbar.update_status_icon(True)
                self.root.geometry('1300x700')
                NewStoryWindow(self.root, self.server, self.robot, self.toolbar)

        else:
            messagebox.showwarning("Aviso!", "Server não encontrado!")

def main():    
    app = AppController()
    
    app.login_window()
    app.root.mainloop()

if __name__ == "__main__":
    main()