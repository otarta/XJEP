import tkinter as tk 

class Main():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("XJEP Control Panel")
        self. scaleFrame = tk.Frame(self) 
        self.scaleFrame.grid(row =1, column = 3, sticky = 'N', columnspan = 4)
        
    
        self.throttleScale = tk.Scale(self.root,
                                      command = self.print_value,
                                      from_ = 100,
                                      to = 0,
                                      orient = "vertical")
        self.throttleScale.pack(padx=10, pady=10)











        self.root.mainloop()
        
    def print_value(self,val):
        print (val)


Main()