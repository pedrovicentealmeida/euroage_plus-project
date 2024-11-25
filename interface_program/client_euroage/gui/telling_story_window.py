from tkinter import Frame, PhotoImage, Label, Button, Text, messagebox, DISABLED, END, NORMAL
from network.client import SocketClientServer, SocketClientRobot
from gui_components.CustomToolbar import CustomToolbar
import threading, time
from utils.helpers import clear_root

class TellingStoryWindow: 
    def __init__(self, root, server: SocketClientServer, robot: SocketClientRobot, toolbar: CustomToolbar):
        self.root = root
        self.server = server
        self.robot = robot
        self.toolbar = toolbar

        self.paused = False
        self.elapsed_time = 0
        
        clear_root(self.root)
        self.create_gui_elements()

    def create_gui_elements(self):
        self.create_controls_frame()
        self.create_timer_frame()
        self.create_interactions_list()

    def create_controls_frame(self):
        controls_frame = Frame(self.root)
        controls_frame.pack(pady=40)

        self.play_img = PhotoImage(file="icons/play.png")
        self.pause_img = PhotoImage(file="icons/pause.png")
        self.stop_img = PhotoImage(file="icons/stop.png")

        self.create_control_btns(controls_frame)
    
    def create_timer_frame(self):
        timer_frame = Frame(self.root)
        timer_frame.pack()

        self.timer_img = PhotoImage(file="icons/clock.png")

        self.create_timer_labels(timer_frame)

    def create_control_btns(self, parent):
        self.play_btn = Button(parent, image=self.play_img, borderwidth=0, activebackground=self.root['bg'], activeforeground=self.root['bg'], command=self.start_story)
        self.pause_btn = Button(parent, image=self.pause_img, borderwidth=0, activebackground=self.root['bg'], activeforeground=self.root['bg'], command=self.pause_story, state=DISABLED)
        self.stop_btn = Button(parent, image=self.stop_img, borderwidth=0, activebackground=self.root['bg'], activeforeground=self.root['bg'], command=self.stop_story, state=DISABLED)

        self.play_btn.grid(row=0, column=1, padx=90)
        self.pause_btn.grid(row=0, column=2, padx=90)
        self.stop_btn.grid(row=0, column=3, padx=90)

    def create_timer_labels(self, parent):
        clock_label = Label(parent, image=self.timer_img)
        self.timer_label = Label(parent, text=": 00:00", font=("Consolas", 13))

        clock_label.grid(row=0, column=1)
        self.timer_label.grid(row=0, column=2)
    
    def create_interactions_list(self):
        self.interactions_list = Text(self.root, width=115, height=18, borderwidth=0, background="gray", wrap="word", font=("Consolas", 12))
        self.interactions_list.pack(pady=20)
        self.interactions_list.config(state=DISABLED)

    def start_story(self):
        self.toolbar.toggle_state("disabled")
        self.paused = False
        self.robot.send_commands_robot("1")
        
        if self.elapsed_time == 0:
            recive_text_thread = threading.Thread(target=self.atualize_text)
            recive_text_thread.start()
        
        self.play_btn.config(state=DISABLED)
        self.pause_btn.config(state="normal")
        self.stop_btn.config(state="normal")
        
        update_timer_thread = threading.Thread(target=self.update_timer)
        update_timer_thread.start()

    def pause_story(self):
        self.robot.send_commands_robot("0")
        
        self.paused = True  # Atualiza a variável 'paused' para True
        self.play_btn.config(state="normal")
        self.pause_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
    
    def stop_story(self):
        response = messagebox.askyesno("Terminar História", "Tem certeza de que deseja terminar a história agora?")
        
        if response:
            self.play_btn.config(state="disabled")
            self.pause_btn.config(state="disabled")
            self.stop_btn.config(state="disabled")
            self.robot.send_commands_robot("-1")
        
    def update_timer(self):
        while True:
            if not self.paused:
                mins, secs = divmod(self.elapsed_time, 60)
                self.timer_label.config(text=f": {mins:02d}:{secs:02d}")
                time.sleep(1)
                self.elapsed_time += 1
            else:
                time.sleep(0.1)
    
    def atualize_text(self):
        while True:
            text = self.robot.receive_text_robot()
            if text:
                if text == "-1":
                    self.paused = True
                    self.play_btn.config(state="disabled")
                    self.pause_btn.config(state="disabled")
                    self.stop_btn.config(state="disabled")
                    messagebox.showwarning("Aviso!", "Fim da história!")
                    self.elapsed_time = 0
                    self.toolbar.toggle_state("normal")
                    break
                elif text == "YES":
                    self.paused = True
                elif text == "-":
                    pass
                else:
                    self.interactions_list.config(state=NORMAL)
                    if text.startswith("Robô"):
                        self.interactions_list.insert(END, f"{text}\n")
                    else:
                        self.interactions_list.insert(END, f"{text}\n")
                    self.interactions_list.config(state=DISABLED)
                    self.interactions_list.see(END)