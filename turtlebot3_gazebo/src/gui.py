#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
import tkinter as tk

class RobotGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("AMR Robot GUI")

        self.status = True
        self.current_table = ''
        self.table_list = []
        self.navigation_status = False
        rospy.init_node("robot_gui_node", anonymous=True)

        # publisher list
        self.command_pub = rospy.Publisher("/gui_commands", String, queue_size=10)
        # subscriber list
        self.robot_status_sub = rospy.Subscriber("/robot_availability", String, self.status_callback)
        self.current_table_sub = rospy.Subscriber('/current_table', String, self.current_table_callback)
        self.current_table_list_sub = rospy.Subscriber('/all_table', String, self.all_table_callback)
        self.navigation_status_sub = rospy.Subscriber('/navigation_status', Bool, self.navigation_status_callback)
        # Gui Init
        self.start_button = tk.Button(master, text="Start Delivery", command=self.start_delivery)
        self.confirm_button = tk.Button(master, text="Confirm Receive", command=self.confirm_receive)

        self.status_label = tk.Label(master, text="Robot Status: Available", fg="green")
        self.status_label.pack(pady=10)

        self.update_buttons()

    def status_callback(self, msg):
        self.status = msg.data.lower() == "true"
        self.update_buttons()

    def current_table_callback(self, msg):
        self.current_table = msg.data
        self.update_buttons()
        # self.table_list.append(msg.data)

    def all_table_callback(self, msg):
        if msg.data not in self.table_list:
            self.table_list.append(msg.data)
        self.update_buttons()

    def navigation_status_callback(self, msg):
        self.navigation_status = msg.data
        self.update_buttons()

    def update_buttons(self):
        # Clear previous buttons
        for widget in self.master.winfo_children():
            if isinstance(widget, tk.Button):
                widget.pack_forget()

        # Update label
        if self.status and len(self.table_list) > 0:
            original_text = "Robot Status: Available\nTable Lists:"
            for table in self.table_list:
                original_text += f'\nTable {table}'
            self.status_label.config(text=f"{original_text}", fg="green")
            self.start_button.pack(pady=10)
        elif self.status and len(self.table_list) == 0:
            self.status_label.config(text="Robot Status: Available", fg="green")
        elif not self.status and self.current_table != '0' and not self.navigation_status:
            self.status_label.config(text=f"Robot Status: Arrived at the Table {self.current_table}", fg="#7B68EE")
            self.confirm_button.pack(pady=10)
        elif not self.status and self.current_table != '0' and self.navigation_status:
            self.status_label.config(text=f"Robot Status: Delivering Orders (Table {self.current_table})", fg="	#FF8C00")
        elif not self.status and self.current_table == '0':
            self.table_list = []
            self.status_label.config(text=f"Robot Status: Return to Home", fg="red")

    def start_delivery(self):
        rospy.loginfo("Start Delivery clicked")
        self.command_pub.publish("start_delivery")

    def confirm_receive(self):
        rospy.loginfo("Confirm Receive clicked")
        self.command_pub.publish("confirm_receive")

if __name__ == '__main__':
    root = tk.Tk()
    gui = RobotGUI(root)
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass