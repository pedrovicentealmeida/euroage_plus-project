##################################################################################
# BSD 3-Clause License
# 
# Copyright (c) 2025, Pedro V. Almeida
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################

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
