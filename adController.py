import random
import serial 



def connection_Status():
    connection = True
    return connection 




def set_Throttle(throttleValue):
    successValue = True
    rpm =throttleValue


def set_ThroatAperture(throatValue):
    successValue = True
    print("Throat Aperture :"+str(throatValue))
    
def set_ExitAperture(exitValue):
    successValue = True
    print("Exit Aperture :"+str(exitValue))



def get_RPM():
    rpm = random.randint(0,10000)
    return rpm

def get_EGT():
    egt = random.randint(0,1200)
    return egt



def set_afterBurner(afterBurnerValue):
    return afterBurnerValue

def set_afterBurnerFuelPump(afterBurnerFuelPumpValue):
    return afterBurnerFuelPumpValue

def set_engine(engineValue):
    return engineValue

def set_fuelPump(fuelPumpValue):
    return fuelPumpValue