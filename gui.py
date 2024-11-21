import tkinter as tk 

class Main():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("XJEP Control Panel")
        self.root.geometry("300x300")
        self. scaleFrame = tk.Frame() 
        self.scaleFrame.grid(row =1, column = 3, sticky = 'N', columnspan = 4)
        
    
        self.throttleScale = tk.Scale(self.root,
                                      command = self.print_value,
                                      from_ = 100,
                                      to = 0,
                                      orient = "vertical")
        self.throttleScale.grid(row=1,column=1)


        self.throatApertureScale = tk.Scale(self.root,
                                      from_ = 100,
                                      to = 0,
                                      orient = "vertical")
        self.throatApertureScale.grid(row=1,column=2)


        self.exitApertureScale = tk.Scale(self.root,
                                      from_ = 100,
                                      to = 0,
                                      orient = "vertical")
        self.exitApertureScale.grid(row=1,column=3)






        self.root.mainloop()
        
    def print_value(self,val):
        print (val)


Main()