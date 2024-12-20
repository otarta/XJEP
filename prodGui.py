import tkinter as tk
from tkinter import ttk
import customtkinter
import math
import adController as ad

class JetEngineControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Jet Engine Control Panel")
        self.geometry("800x600")
        self.configure_grid()
        self.setGlobalVars()  
        self.create_widgets()
        self.update_gauge_data()  
        self.update_gauges()      

    def configure_grid(self):
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)  
        self.rowconfigure(2, weight=1)


    def setGlobalVars(self):
        self.throttle = tk.DoubleVar()
        self.throat_aperture = tk.DoubleVar()
        self.exit_aperture = tk.DoubleVar()
        self.rpm = tk.DoubleVar(value=self.getRPM())
        self.egt = tk.DoubleVar(value=0)
        self.fuelPumpState = customtkinter.BooleanVar(value = "false")
        self.engineState = customtkinter.BooleanVar(value = "false")
        self.afterBurnerState = customtkinter.BooleanVar(value = "false")
        self.afterBurnerFuelPumpState = customtkinter.BooleanVar(value = "false")

    def create_widgets(self):
        # Throttle Scale
        throttle_frame = ttk.LabelFrame(self, text="Throttle")
        throttle_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        throttle_frame.columnconfigure(0, weight=1)
        
        ttk.Scale(throttle_frame, from_=0, to=100, command=self.setThrottle, variable=self.throttle, orient="horizontal").grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        ttk.Label(throttle_frame, text="0%").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Label(throttle_frame, text="100%").grid(row=1, column=0, sticky="e", padx=5)


        # Throat Nozzle Aperture Scale
        throat_frame = ttk.LabelFrame(self, text="Throat Nozzle Aperture")
        throat_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        throat_frame.columnconfigure(0, weight=1)
        
        ttk.Scale(throat_frame, from_=0, to=100, command=self.setThroatAperture, variable=self.throat_aperture, orient="horizontal").grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        ttk.Label(throat_frame, text="0%").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Label(throat_frame, text="100%").grid(row=1, column=0, sticky="e", padx=5)
        
        # Exit Nozzle Aperture Scale
        exit_frame = ttk.LabelFrame(self, text="Exit Nozzle Aperture")
        exit_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")
        exit_frame.columnconfigure(0, weight=1)
        
        ttk.Scale(exit_frame, from_=0, to=100, command=self.setExitAperture, variable=self.exit_aperture, orient="horizontal").grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        ttk.Label(exit_frame, text="0%").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Label(exit_frame, text="100%").grid(row=1, column=0, sticky="e", padx=5)
        
        # Gauges
        gauges_frame = ttk.LabelFrame(self, text="Gauges")
        gauges_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky="nsew")
        gauges_frame.columnconfigure(0, weight=1)
        gauges_frame.columnconfigure(1, weight=1)

        self.rpm_canvas = self.create_gauge(gauges_frame, "RPM", 0, 10000, 1000)
        self.egt_canvas = self.create_gauge(gauges_frame, "EGT (°C)", 1, 1200, 200)

        # Switches
        switches_frame = ttk.LabelFrame(self, text="Switches")
        switches_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=10, sticky="nsew")
        switches_frame.columnconfigure(0, weight=1)
        switches_frame.columnconfigure(1, weight=1)
        switches_frame.columnconfigure(2, weight=1)
        switches_frame.columnconfigure(3, weight=1)

        

        self.create_switch(switches_frame, "Fuel Pump", 0, self.setFuelPumpState, self.fuelPumpState)
        self.create_switch(switches_frame, "Engine", 1, self.setEngineState,self.engineState)
        self.create_switch(switches_frame, "Afterburner Fuel Pump", 2, self.setAfterBurnerFuelPumpState, self.afterBurnerFuelPumpState)
        self.create_switch(switches_frame, "Afterburner", 3, self.setAfterBurnerState,self.afterBurnerState)
        
    def create_gauge(self, parent, label, column, max_value, step):
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, padx=10, pady=10, sticky="nsew")
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(column, weight=1)
        
        text = tk.Label(master = frame, text = label)
        text.pack()

        canvas = tk.Canvas(frame, bg="white")
        canvas.pack(fill="both", expand=True)

        frame.bind("<Configure>", lambda event: self.resize_gauge(canvas, event.width, event.height, label, max_value, step))
        return canvas

    def resize_gauge(self, canvas, width, height, label, max_value, step):
        canvas.delete("all")
        size = min(width, height)
        margin = size * 0.1
        center_x = width // 2
        center_y = (height//1.25) - margin 
        radius = size // 2 - margin

        # Draw gauge background
        canvas.create_arc(
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            start=0, extent=180, outline="black", style="arc", width=2
        )

        # Draw labels
        for value in range(0, max_value + 1, step):
            angle = 180 * (value / max_value)
            x = center_x + (radius + margin * 0.65) * math.cos(math.radians(180 - angle))
            y = center_y - (radius + margin * 0.4) * math.sin(math.radians(180 - angle))
            canvas.create_text(x, y, text=str(value), font=("Arial", int(margin * 0.4)), fill="black")

        canvas.create_text(center_x, center_y + margin, text=label, anchor="n")

    def create_switch(self, parent, label, column, command, variable):
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, padx=10, pady=10, sticky="nsew")
        ttk.Label(frame, text=label).pack()
        switch = customtkinter.CTkSwitch(master = frame,command = command,variable = variable, text="",width = 100 , height = 50,switch_width= 100, switch_height=50)
        switch.pack()


    def update_gauge_data(self):
        """Update the RPM and EGT values every second."""
        # Simulate new data
        self.rpm.set(self.getRPM())
        self.egt.set(self.getEGT())
        
        # Schedule the next data update
        self.after(100, self.update_gauge_data)

    def update_gauges(self):
        """Redraw the gauges with the latest data."""
        # Update RPM gauge
        self.draw_gauge_needle(self.rpm_canvas, self.rpm.get() / 10000)
        
        # Update EGT gauge
        self.draw_gauge_needle(self.egt_canvas, self.egt.get() / 1200)

        # Schedule the next gauge update
        self.after(100, self.update_gauges)

    def draw_gauge_needle(self, canvas, normalized_value):
        """Draw the needle on a gauge."""
        canvas.delete("needle")
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        size = min(width, height)
        margin = size * 0.1
        center_x = width // 2
        center_y = (height//1.15)- margin
        radius = size // 2 - margin
        angle = 180 * normalized_value
        end_x = center_x + radius * math.cos(math.radians(180 - angle))
        end_y = center_y - radius * math.sin(math.radians(180 - angle))

        # Draw the needle
        canvas.create_line(center_x, center_y, end_x, end_y, fill="red", width=2, tags="needle")

    def getRPM(self):
        return ad.get_RPM()

    def getEGT(self):
        return ad.get_EGT()

    def setThrottle(self, val):
        ad.set_Throttle(val)

    def setThroatAperture(self, val):
        ad.set_ThroatAperture(val)

    def setExitAperture(self, val):
        ad.set_ExitAperture(val)



    def setFuelPumpState(self):
        val = self.fuelPumpState.get()
        ad.set_fuelPump(val)

    def setEngineState(self):
       val = self.engineState.get()
       ad.set_engine(val)

    def setAfterBurnerState(self):
        val = self.afterBurnerState.get()
        ad.set_afterBurner(val)

    def setAfterBurnerFuelPumpState(self):
        val = self.afterBurnerFuelPumpState.get()
        ad.set_afterBurnerFuelPump(val)









# Run the app
if __name__ == "__main__":
    app = JetEngineControlPanel()
    app.mainloop()
