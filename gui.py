import tkinter as tk 

class Main():
    def __init__(self):

        self.root = tk.Tk()
        self.root.title("XJEP Control Panel")

        self.throttleVal =  tk.DoubleVar()

        self.throttleScale = tk.Scale(self.root,
                                      variable = self.throttleVal,
                                      command = self.printVal,
                                      from_ = 100,
                                      to = 0,
                                      orient = "vertical")
        self.throttleScale.pack(padx=10, pady=10)

        self.root.mainloop()
        
    def printVal(self):
        print (self.throttleVal.get())

Main()