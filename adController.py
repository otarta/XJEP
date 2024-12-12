import random
import serial 



def connection_Status():
    connection = True
    return connection 


def set_Throttle(throttleValue):
    print("Throttle : "+str(throttleValue))

def set_ThroatAperture(throatValue):
    print("Throat Aperture :"+str(throatValue))
    
def set_ExitAperture(exitValue):
    print("Exit Aperture :"+str(exitValue))



def get_RPM():
    rpm = random.randint(0,10000)
    return rpm

def get_EGT():
    egt = random.randint(0,1200)
    return egt



def set_fuelPump(fuelPumpValue):
    print ("Fuel Pump State: ", fuelPumpValue)
    return fuelPumpValue

def set_engine(engineValue):
    print ("Engine State: ", engineValue)
    return engineValue

def set_afterBurnerFuelPump(afterBurnerFuelPumpValue):
    print ("AfterBurner Fuel Pump State: ", afterBurnerFuelPumpValue)
    return afterBurnerFuelPumpValue

def set_afterBurner(afterBurnerValue):
    print ("AfterBurner State: ", afterBurnerValue)
    return afterBurnerValue





