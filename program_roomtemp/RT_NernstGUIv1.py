#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created: 2015-05-29

@author: Bobby McKinney (bobbymckinney@gmail.com)

__Title__ : room temp nernst
Description:
Comments:
"""
import os
import sys
import wx
from wx.lib.pubsub import pub # For communicating b/w the thread and the GUI
import matplotlib
matplotlib.interactive(False)
matplotlib.use('WXAgg') # The recommended way to use wx with mpl is with WXAgg backend.

from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
from matplotlib.figure import Figure
from matplotlib.pyplot import gcf, setp
import matplotlib.animation as animation # For plotting
import pylab
import numpy as np
import matplotlib.pyplot as plt
import serial # for communicating with Lakeshore magnet power supply
import visa # pyvisa, essential for communicating with the Keithley
from threading import Thread # For threading the processes going on behind the GUI
import time
from datetime import datetime # for getting the current date and time
import minimalmodbus as modbus # For communicating with the cn7500s
import omegacn7500 # Driver for cn7500s under minimalmodbus, adds a few easy commands

# Modules for saving logs of exceptions
import exceptions
import sys
from logging_utils import setup_logging_to_file, log_exception

# for a fancy status bar:
import EnhancedStatusBar as ESB

# for getting magnetic field value
from MagFieldInterpolator import fieldInterp

# For finding sheet resistance:
#import RT_Seebeck_Processing_v1

#==============================================================================
# Keeps Windows from complaining that the port is already open:
modbus.CLOSE_PORT_AFTER_EACH_CALL = True


version = '1.0 (2015-06-16)'

'''
Global Variables:
'''

# Naming a data file:
dataFile = 'Data_Backup.csv'
statusFile = 'Status.csv'
finaldataFile = 'Data.csv'

APP_EXIT = 1 # id for File\Quit

maxLimit = 70 # Restricts the user to a max temperature

abort_ID = 0 # Abort method

# Global placers for instruments
k2700 = ''
k2182 = ''
heaterA = ''
heaterB = ''
ls643 = ''

maxMagCurrent = 70 # (A) Restricts the user to a max current on mag power supply
maxMagRate = 40 # (A/s) Restricts the user to a max current ramp rate on mag power supply

magcurrent = 70 # (A) Current that is supplied by the mag power supply

magcurlist = [float(x) for x in range(0,70,5)+range(70,-70,-5)+range(-70,0,5)]
magrate = 30 # (A) Current ramp rate that is supplied to the mag power supply
APP_EXIT = 1 # id for File\Quit
stability_threshold = 0.10/60.0 # change in PID temp must be less than this value for a set time in order to reach an equilibrium
tolerance = 0.5 # Temperature must be within this temperature range of the PID setpoint in order to begin a measurement
dTlist = [-18.0, -14.0, -10.0, -6.0, -2.0, 0.0, 2.0, 6.0, 10.0, 14.0, 18.0]

xdist = 10.0 # placeholder for sample thickness in dT direction
ydist = 10.0 # placeholder for sample thicnkess in V direction
zdist = 15.240 # placeholder for distance between magnet poles in mm

# placer for directory
filePath = 'global file path'

# placer for files to be created
myfile = 'global file'
rawfile = 'global file'

# Placers for the GUI plots:
VN_list = []
tVN_list = []
VS_list = []
tVS_list = []
tempA_list = []
ttempA_list = []
tempB_list = []
ttempB_list = []
B_list = []
tB_list = []

#ResourceManager for visa instrument control
ResourceManager = visa.ResourceManager()

###############################################################################
class Keithley_2700:
    ''' Used for the matrix card operations. '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)
        self.ctrl.write("sens:func 'volt:dc'")
        self.ctrl.write("sens:volt:dc:range:auto on")
        self.ctrl.write("sens:volt:dc:NPLCycles 5")

    #end init

    #--------------------------------------------------------------------------
    def fetch(self):
        """
        Scan the channel and take a reading
        """

        data = self.ctrl.query("fetch?")
        time.sleep(0.1)
        vdc = data.split(",")[0]
        v = float(vdc[:-3])
        return v # Fetches Reading
    #end def

#end class
###############################################################################

###############################################################################
class Keithley_2182:
    ''' NanoVoltMeter '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)
        #self.ctrl.write("*RST") #resets k2182
        self.ctrl.write(":TRIGger:SEQuence1:COUNt 1")
        self.ctrl.write(":TRIGger:SEQuence1:DELay 0") # Set count rate
        self.ctrl.write(":SENSe:FUNCtion VOLTage")
        self.ctrl.write(":SENS:VOLT:CHAN1:RANG:AUTO ON")
        self.ctrl.write(":SENSe1:VOLTage:DC:NPLCycles 5") # Sets integration period based on frequency


    #end init

    #--------------------------------------------------------------------------
    def fetch(self):
        """
        Scan the channel and take a reading
        """
        #self.write(":ROUTe:SCAN:INTernal:CCOunt 1") # Specify number of readings on channel 1
        self.ctrl.write(":SENSe:CHANnel 1")
        data = self.ctrl.query(":SENSe:DATA:FRESh?")
        #print str(data)[0:15]
        #print data
        return float(data[0:-1]) # Fetches Reading

    #end def

#end class
###############################################################################

###############################################################################
class Lakeshore_643(serial.Serial):
    '''Electromagnet Power Supply'''
    #--------------------------------------------------------------------------
    def __init__(self, port):
        serial.Serial.__init__(self, port, 57600,serial.SEVENBITS,serial.PARITY_ODD, serial.STOPBITS_ONE, None,False,False,None,False,None)

        self.term = '\r\n'
        self.delay = 2

        # clear error
        self.clearError()

        # clear and reset power supply
        self.clearInstrument()
        self.resetInstrument()

        #set current to 0
        self.setCurrent(0)

    #end init

    def setCurrent(self, current):
        if (-70 <= current <= 70):
            self.write('SETI ' + str(current) + self.term)
        else: pass
    #end def

    def getCurrent(self):
        self.write('RDGI? ' + self.term)
        time.sleep(self.delay)
        current = self.readline()

        current = float(current[:-2])
        return current
    #end def

    def getVoltage(self):
        self.write('RDGV? ' + self.term)
        time.sleep(self.delay)
        voltage = self.readline()
        voltage = float(voltage[:-2])
        return voltage
    #end def

    def setRampRate(self, rate):
        if (0.0001 <= rate <= 50):
            self.write('RATE ' + str(rate) + ',' + self.term)
        else: pass
    #end def

    def getRampRate(self):
        self.write('RATE? ' + self.term)
        time.sleep(self.delay)
        rate = self.readline()
        rate = float(rate[:-2])
        return rate
    #end def

    def stopRamp(self):
        self.write('STOP' + self.term)
        time.sleep(20*self.delay)
    #end def

    def setLimits(self, currentLim, rateLim):
        if (0.0001 <= rateLim <= 50 and 0 <= currentLim <= 70.1000):
            self.write('LIMIT' + str(currentLim) + ',' + str(rateLim) + self.term)
        else: pass
    #end def

    def getLimits(self):
        self.write('LIMIT?' + self.term)
        time.sleep(self.delay)
        limits = self.readline()
        limits = limits[:-2]
        limits = limits.split(',')
        currentLim = float(limits[0])
        rateLim = float(limits[1])
        return currentLim, rateLim
    #end def

    def errorTest(self):
        self.write('*TST?' + self.term)
        time.sleep(self.delay)
        error = self.readline()
        error = int(error[:-2])
        return error
    #end def

    def clearError(self):
        self.write('ERCL' + self.term)
    #end def

    def clearInstrument(self):
        self.write('*CLS' + self.term)
    #end def

    def resetInstrument(self):
        self.write('*RST' + self.term)
    #end def

    def factorySettings(self):
        self.write('DFLT 99' + self.term)
    #end def

#end class
###############################################################################

###############################################################################
class PID(omegacn7500.OmegaCN7500):

    #--------------------------------------------------------------------------
    def __init__(self, portname, slaveaddress):
        omegacn7500.OmegaCN7500.__init__(self, portname, slaveaddress)

    #end init

    #--------------------------------------------------------------------------

    # Commands for easy reference:
    #    Use .write_register(command, value) and .read_register(command)
    #    All register values can be found in the Manual or Instruction Sheet.
    #    You must convert each address from Hex to Decimal.
    control = 4101 # Register for control method
    pIDcontrol = 0 # Value for PID control method
    pIDparam = 4124 # Register for PID parameter selection
    pIDparam_Auto = 4 # Value for Auto PID
    tCouple = 4100 # Register for setting the temperature sensor type
    tCouple_K = 0 # K type thermocouple
    heatingCoolingControl = 4102 # Register for Heating/Cooling control selection
    heating = 0 # Value for Heating setting

#end class
###############################################################################

###############################################################################
class Setup:
    """
    Call this class to run the setup for the Keithley and the PID.
    """
    def __init__(self):
        """
        Prepare the Keithley to take data on the specified channels:
        """
        global k2700
        global k2182
        global heaterA
        global heaterB
        global ls643

        global maxMagCurrent
        global maxMagRate

        # Define Keithley instrument port:
        self.k2700 = k2700 = Keithley_2700('GPIB0::2::INSTR') # MultiMeter for Matrix Card operation
        self.k2182 = k2182 = Keithley_2182('GPIB0::4::INSTR') # NanoVoltMeter
        # Define the ports for the PID
        self.heaterA = heaterA = PID('/dev/cu.usbserial', 1) # heater A
        self.heaterB = heaterB = PID('/dev/cu.usbserial', 2) # heater B
        # define the port for the magnet power supply
        self.ls643 = ls643 = Lakeshore_643('/dev/cu.SLAB_USBtoUART') # magnet power supply

        """
        Prepare the PID for operation:
        """
        # Set the control method to PID
        self.heaterA.write_register(PID.control, PID.pIDcontrol)
        self.heaterB.write_register(PID.control, PID.pIDcontrol)

        # Set the PID to auto parameter
        self.heaterA.write_register(PID.pIDparam, PID.pIDparam_Auto)
        self.heaterB.write_register(PID.pIDparam, PID.pIDparam_Auto)

        # Set the thermocouple type
        self.heaterA.write_register(PID.tCouple, PID.tCouple_K)
        self.heaterB.write_register(PID.tCouple, PID.tCouple_K)

        # Set the control to heating only
        self.heaterA.write_register(PID.heatingCoolingControl, PID.heating)
        self.heaterB.write_register(PID.heatingCoolingControl, PID.heating)

        # Run the controllers
        self.heaterA.run()
        self.heaterB.run()

        """
        Set Limits on Magnet Power Supply
        """
        self.ls643.clearInstrument()
        self.ls643.factorySettings()
        self.ls643.setLimits(maxMagCurrent,maxMagRate)

        """
        Set Reference voltage to 0
        """
        #self.k2182.ctrl.write(":SENSe1:VOLTage:DC:REFerence:ACQuire")
        #self.k2182.ctrl.write(":SENSe1:VOLTage:DC:REFerence:STATe ON")

#end class
###############################################################################

###############################################################################
class ProcessThreadRun(Thread):
    """
    Thread that runs the operations behind the GUI. This includes measuring
    and plotting.
    """

    #--------------------------------------------------------------------------
    def __init__(self):
        """ Init Worker Thread Class """
        Thread.__init__(self)
        self.start()

    #end init

    #--------------------------------------------------------------------------
    def run(self):
        """ Run Worker Thread """
        #Setup()
        td=TakeData()
        #td = TakeDataTest()
    #end def

#end class
###############################################################################

###############################################################################
class InitialCheck:
    def __init__(self):
        global k2182
        global k2700
        global heaterA
        global heaterB
        global ls643

        global zdist

        self.heaterA = heaterA
        self.heaterB = heaterB
        self.ls643 = ls643
        self.k2182 = k2182
        self.k2700 = k2700

        self.getPIDdata()

        self.getMagdata()

        self.getVdata()

        self.InitialSettings()
    #end init

    #--------------------------------------------------------------------------
    def getPIDdata(self):
        self.tempA = float(self.heaterA.get_pv())
        self.tempAset = float(self.heaterA.get_setpoint())
        self.tempB = float(self.heaterB.get_pv())
        self.tempBset = float(self.heaterB.get_setpoint())

        print "temp A: %f C\ntemp A SP: %f C\ntemp B: %f C\ntemp B SP: %f C\n" %(self.tempA, self.tempAset, self.tempB, self.tempBset)

        self.updateGUI(stamp="Temp A Status", data=self.tempA)
        self.updateGUI(stamp="Temp A SP Status", data=self.tempAset)
        self.updateGUI(stamp="Temp B Status", data=self.tempB)
        self.updateGUI(stamp="Temp B SP Status", data=self.tempBset)
    #end def

    #--------------------------------------------------------------------------
    def getMagdata(self):
        self.magcurrent = self.ls643.getCurrent()
        self.magrate = self.ls643.getRampRate()
        self.z = float(zdist)
        self.B = fieldInterp(self.z, self.magcurrent)

        print "mag current: %f A\nmag rate: %f A/s\nmag field: %f T\n" %(self.magcurrent, self.magrate, self.B)

        self.updateGUI(stamp="Mag Current Status", data=self.magcurrent)
        self.updateGUI(stamp="Mag Rate Status", data=self.magrate)
        self.updateGUI(stamp="Bfield Status", data=self.B)
    #end def

    #--------------------------------------------------------------------------
    def getVdata(self):
        self.VN =  self.k2182.fetch()
        time.sleep(1)
        self.VS = self.k2700.fetch()

        print "voltage (nernst): %f" % (self.VN)
        print "voltage (seebeck): %f" % (self.VS)
        #self.V = float(self.k2182.fetch())*10**6 # uV

        self.updateGUI(stamp="Nernst Voltage Status", data=float(self.VN))
        self.updateGUI(stamp="Seebeck Voltage Status", data=float(self.VS))
    #end def

    #--------------------------------------------------------------------------
    def InitialSettings(self):
        print "set tempA = tempB = 30 C, dT = 0"
        print "set mag current = 0"
        self.ls643.setCurrent(0)
        self.heaterA.set_setpoint(30)
        self.heaterB.set_setpoint(30)
    #end def

    #--------------------------------------------------------------------------
    def updateGUI(self, stamp, data):
        """
        Sends data to the GUI (main thread), for live updating while the process is running
        in another thread.
        """
        time.sleep(0.1)
        wx.CallAfter(pub.sendMessage, stamp, msg=data)
    #end def
#end class
###############################################################################

###############################################################################
class TakeData:
    ''' Takes measurements and saves them to file. '''
    #--------------------------------------------------------------------------
    def __init__(self):
        global abort_ID
        global k2182
        global k2700
        global heaterA
        global heaterB
        global ls643

        global tolerance
        global stability_threshold
        global dTlist

        global xdist
        global ydist
        global zdist

        global magrate
        global magcurlist

        self.k2182 = k2182
        self.k2700 = k2700
        self.heaterA = heaterA
        self.heaterB = heaterB
        self.ls643 = ls643
        self.k2182.ctrl.write(":SENSe1:VOLTage:DC:NPLCycles 5") # Sets integration period based on frequency

        self.x = float(xdist)
        self.y = float(ydist)
        self.z = float(zdist)

        self.magcurrent = self.ls643.getCurrent()

        self.tolerance = tolerance
        self.stability_threshold = stability_threshold

        self.delay = 1
        self.magdelay = 5
        self.tempdelay = 5

        self.tol = 'NO'
        self.stable = 'NO'
        self.measurement = 'OFF'
        self.updateGUI(stamp='Measurement', data=self.measurement)

        #time initializations
        self.ttempA = 0
        self.ttempA2 = 0
        self.ttempB = 0
        self.ttempB2 = 0
        self.tVN = 0
        self.tVN2 = 0
        self.tVS = 0
        self.tVS2 = 0
        self.tB = 0

        self.exception_ID = 0

        self.updateGUI(stamp='Status Bar', data='Running')

        self.start = time.time()

        try:
            while abort_ID == 0:

                # vary dT
                for dT in dTlist:
                    print "Set dT to %f" %(dT)
                    self.heaterA.set_setpoint(45.0+dT/2.0)
                    self.heaterB.set_setpoint(45.0-dT/2.0)
                    self.recenttempA = []
                    self.recenttempAtime=[]
                    self.recenttempB = []
                    self.recenttempBtime=[]
                    self.stabilityA = '-'
                    self.stabilityB = '-'
                    self.updateGUI(stamp="Stability A", data=self.stabilityA)
                    self.updateGUI(stamp="Stability B", data=self.stabilityB)
                    self.tempAset = float(self.heaterA.get_setpoint())
                    self.tempBset = float(self.heaterB.get_setpoint())
                    self.take_PID_Data()
                    # ramp to correct dT
                    while (self.tol != 'OK' or self.stable != 'OK'):
                        self.take_PID_Data()
                        self.updateStats()

                        time.sleep(self.tempdelay)
                        if abort_ID == 1: break
                    # end while
                    if abort_ID == 1: break
                    # start measurement
                    if (self.tol == 'OK' and self.stable == 'OK'):
                        for m in range(10):
                            self.updateStats()
                            time.sleep(self.tempdelay)
                            if abort_ID == 1: break
                        self.measurement = 'ON'
                        self.updateGUI(stamp='Measurement', data=self.measurement)
                        #vary mag current
                        for cur in magcurlist:
                            self.magfield(float(cur))
                            # take voltage data
                            self.data_measurement()
                            self.write_data_to_file()
                            self.updateStats()
                            if abort_ID == 1: break

                        # end for
                        if abort_ID == 1: break
                        self.magfield(0)
                        self.measurement = 'OFF'
                        for m in range(20):
                            self.updateStats()
                            time.sleep(self.tempdelay)
                            if abort_ID == 1: break
                        self.tol = 'NO'
                        self.stable = 'NO'
                        self.updateGUI(stamp='Measurement', data=self.measurement)
                    #end if
                    if abort_ID == 1: break
                #end for
                abort_ID = 1
            #end while
        #end try

        except exceptions.Exception as e:
            log_exception(e)

            abort_ID = 1

            self.exception_ID = 1

            print "Error Occurred, check error_log.log"
        #end except

        if self.exception_ID == 1:
            self.updateGUI(stamp='Status Bar', data='Exception Occurred')
        #end if
        else:
            self.updateGUI(stamp='Status Bar', data='Finished, Ready')
        #end else

        self.heaterA.set_setpoint(30)
        self.heaterB.set_setpoint(30)
        self.ls643.setCurrent(0)

        self.save_files()

        wx.CallAfter(pub.sendMessage, 'Post Process')
        wx.CallAfter(pub.sendMessage, 'Enable Buttons')

    #end init

    #--------------------------------------------------------------------------
    def safety_check(self):
        global maxLimit
        global abort_ID

        if float(self.tempA) > maxLimit or float(self.tempB) > maxLimit:
            abort_ID = 1
    #end def

    #--------------------------------------------------------------------------
    def take_PID_Data(self):
        """ Takes data from the PID and proceeds to a
            function that checks the PID setpoints.
        """

        try:
            # Take Data
            self.tempA = float(self.heaterA.get_pv())
            self.ttempA = time.time() - self.start
            self.tempB = float(self.heaterB.get_pv())
            self.ttempB = time.time() - self.start

            # Get the current setpoints on the PID:
            self.tempAset = float(self.heaterA.get_setpoint())
            self.tempBset = float(self.heaterB.get_setpoint())

        except exceptions.ValueError as VE:
            print(VE)
            self.tempA = float(self.heaterA.get_pv())
            self.ttempA = time.time() - self.start
            self.tempB = float(self.heaterB.get_pv())
            self.ttempB = time.time() - self.start

            # Get the current setpoints on the PID:
            self.tempAset = float(self.heaterA.get_setpoint())
            self.tempBset = float(self.heaterB.get_setpoint())



        #check stability of PID
        if (len(self.recenttempA)<10):
            self.recenttempA.append(self.tempA)
            self.recenttempAtime.append(self.ttempA)
            self.recenttempB.append(self.tempB)
            self.recenttempBtime.append(self.ttempB)

        else:
            self.recenttempA.pop(0)
            self.recenttempAtime.pop(0)
            self.recenttempA.append(self.tempA)
            self.recenttempAtime.append(self.ttempA)
            self.recenttempB.pop(0)
            self.recenttempBtime.pop(0)
            self.recenttempB.append(self.tempB)
            self.recenttempBtime.append(self.ttempB)

            self.stabilityA = self.getStability(self.recenttempA,self.recenttempAtime)
            #self.stabilityA = float((self.recenttempA[-1]-self.recenttempA[0])/(self.recenttempAtime[-1]-self.recenttempAtime[0]))
            print "stability A: %.4f C/min" % (self.stabilityA*60)
            self.stabilityB = self.getStability(self.recenttempB,self.recenttempBtime)
            #self.stabilityB = float((self.recenttempB[-1]-self.recenttempB[0])/(self.recenttempBtime[-1]-self.recenttempBtime[0]))
            print "stability B: %.4f C/min" % (self.stabilityB*60)
            self.updateGUI(stamp="Stability A", data=self.stabilityA*60)
            self.updateGUI(stamp="Stability B", data=self.stabilityB*60)





        self.check_status()

        self.safety_check()
    #end def

    #--------------------------------------------------------------------------
    def getStability(self, temps, times):
        coeffs = np.polyfit(times, temps, 1)

        # Polynomial Coefficients
        results = coeffs.tolist()
        return results[0]
    #end def

    #--------------------------------------------------------------------------
    def updateStats(self):
        print('update all stats\n')

        self.updateGUI(stamp="Time Temp A", data=self.ttempA)
        self.updateGUI(stamp="Temp A", data=self.tempA)
        self.updateGUI(stamp="Temp A SP", data=self.tempAset)

        self.updateGUI(stamp="Time Temp B", data=self.ttempB)
        self.updateGUI(stamp="Temp B", data=self.tempB)
        self.updateGUI(stamp="Temp B SP", data=self.tempBset)

        print "tempA: %s C\ntempB: %s C" % (self.tempA, self.tempB)
        self.dT = self.tempA - self.tempB

        self.magcurrent = self.ls643.getCurrent()
        self.magrate = self.ls643.getRampRate()
        self.tB = time.time()-self.start
        self.B = fieldInterp(self.z, self.magcurrent)

        print "mag current: %f\nmag field: %f" % (self.magcurrent, self.B)

        self.updateGUI(stamp="Mag Current", data=self.magcurrent)
        self.updateGUI(stamp="Mag Rate", data=self.magrate)
        self.updateGUI(stamp="Time Bfield", data=self.tB)
        self.updateGUI(stamp="Bfield", data=self.B)

        self.VN = float(self.k2182.fetch())
        self.tVN = time.time() - self.start

        self.VS = float(self.k2700.fetch())
        self.VScalc = self.alpha(self.tempA,self.tempB)*self.dT - self.VS
        self.tVS = time.time() - self.start

        print "nernst voltage: %.2f\nseebeck voltage: %.2f" % (self.VN*10**6, self.VScalc*10**6)

        self.updateGUI(stamp="Time Nernst Voltage", data=float(self.tVN))
        self.updateGUI(stamp="Nernst Voltage", data=float(self.VN))
        self.updateGUI(stamp="Time Seebeck Voltage", data=float(self.tVS))
        self.updateGUI(stamp="Seebeck Voltage", data=float(self.VScalc))

        global rawfile
        print('\nWrite status to file\n')
        rawfile.write('%.1f,'%(self.tVN))
        rawfile.write('%.4f,%.4f,'%(self.x,self.y))
        rawfile.write('%.2f,%.2f,'%(self.tempA, self.tempAset))
        rawfile.write('%.2f,%.2f,'%(self.tempB, self.tempBset))
        rawfile.write('%.2f,%.2f,%.4f,'%(self.magrate, self.magcurrent, self.B))
        rawfile.write('%.3f,'%(self.VN*10**6))
        rawfile.write('%.3f\n'%(self.VScalc*10**6))
    #end def

    #--------------------------------------------------------------------------
    def check_status(self):

        if (np.abs(self.tempA-self.tempAset) < self.tolerance and np.abs(self.tempB-self.tempBset) < self.tolerance):

            self.tol = 'OK'
        #end if

        else:
            self.tol = 'NO'

        #end else
        if (self.stabilityA != '-' and self.stabilityB != '-'):
            if (np.abs(self.stabilityA) < self.stability_threshold and np.abs(self.stabilityB) < self.stability_threshold):
                self.stable = 'OK'
            #end if
            else:
                self.stable = 'NO'
        #end if
        else:
            self.stable = 'NO'

        #end else

        print "tolerance: %s\nstable: %s\n" % (self.tol, self.stable)
            #end else
        #end elif

        self.updateGUI(stamp="Status Bar", data=[self.tol, self.stable])
    #end def

    #--------------------------------------------------------------------------
    def magfield(self,cur):
        global magrate
        self.B = 4
        while np.abs(self.B) > 3:
            self.ls643.setCurrent(cur)
            print "set mag current to %f\n" % (cur)
            time.sleep(self.magdelay)
            #end else
            self.magcurrent = self.ls643.getCurrent()
            self.magrate = self.ls643.getRampRate()
            self.tB = time.time()-self.start

            self.B = fieldInterp(self.z, self.magcurrent)
            cur = cur+1

        self.updateGUI(stamp="Mag Current", data=self.magcurrent)
        self.updateGUI(stamp="Mag Rate", data=self.magrate)
        self.updateGUI(stamp="Time Bfield", data=self.tB)
        self.updateGUI(stamp="Bfield", data=self.B)

    #end def

    #--------------------------------------------------------------------------
    def data_measurement(self):
        # get temp A
        print('data for calculation\n')
        self.magcurrent = self.ls643.getCurrent()
        self.magrate = self.ls643.getRampRate()
        self.tB = time.time()-self.start
        self.B = fieldInterp(self.z, self.magcurrent)

        self.tempA = float(self.heaterA.get_pv())
        self.ttempA = time.time() - self.start
        self.updateGUI(stamp="Temp A", data=float(self.tempA))
        self.updateGUI(stamp="Time Temp A", data=float(self.ttempA))
        print "ttempA: %.2f s\ttempA: %s C" % (self.ttempA, self.tempA)

        time.sleep(self.delay)

        # get temp B
        self.tempB = float(self.heaterB.get_pv())
        self.ttempB = time.time() - self.start
        self.updateGUI(stamp="Temp B", data=float(self.tempB))
        self.updateGUI(stamp="Time Temp B", data=float(self.ttempB))
        print "ttempB: %.2f s\ttempB: %s C" % (self.ttempB, self.tempB)

        time.sleep(self.delay)

        self.dT = self.tempA - self.tempB

        # get high V
        v = [0,0,0,0,0]
        v[0] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[1] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[2] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[3] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[4] = float(self.k2182.fetch())
        self.VN = np.average(v)
        self.tVN = time.time() - self.start
        self.updateGUI(stamp="Time Nernst Voltage", data=float(self.tVN))
        self.updateGUI(stamp="Nernst Voltage", data=float(self.VN))

        print "tVN: %.2f s\tVN: %.2f V" % (self.tVN, self.VN*10**6)

        time.sleep(self.delay)

        self.VS = float(self.k2700.fetch())
        self.VScalc = self.alpha(self.tempA,self.tempB)*self.dT - self.VS
        self.tVS = time.time() - self.start
        self.updateGUI(stamp="Time Seebeck Voltage", data=float(self.tVS))
        self.updateGUI(stamp="Seebeck Voltage", data=float(self.VScalc))

        print "tVS: %.2f s\tVS: %.2f V" % (self.tVS, self.VScalc*10**6)

        time.sleep(self.delay)

        # Symmetrize the measurement and repeat in reverse

        self.VS2 = float(self.k2700.fetch())
        self.VScalc2 = self.alpha(self.tempA,self.tempB)*self.dT - self.VS2
        self.tVS2 = time.time() - self.start
        self.updateGUI(stamp="Time Seebeck Voltage", data=float(self.tVS2))
        self.updateGUI(stamp="Seebeck Voltage", data=float(self.VScalc2))

        print "tVS: %.2f s\tVS: %.2f V" % (self.tVS2, self.VScalc2*10**6)

        time.sleep(self.delay)

        v = [0,0,0,0,0]
        v[0] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[1] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[2] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[3] = float(self.k2182.fetch())
        time.sleep(self.delay/2.0)
        v[4] = float(self.k2182.fetch())
        self.VN2 = np.average(v)
        self.tVN2 = time.time() - self.start
        self.updateGUI(stamp="Time Voltage", data=float(self.tVN2))
        self.updateGUI(stamp="Voltage", data=float(self.VN2))

        print "tVN: %.2f s\tVN: %f V" % (self.tVN2, self.VN2*10**6)

        time.sleep(self.delay)

        self.tempB2 = float(self.heaterB.get_pv())
        self.ttempB2 = time.time() - self.start
        self.updateGUI(stamp="Temp B", data=float(self.tempB2))
        self.updateGUI(stamp="Time Temp B", data=float(self.ttempB2))
        print "ttempB: %.2f s\ttempB: %s C" % (self.ttempB2, self.tempB2)

        time.sleep(self.delay)

        self.tempA2 = float(self.heaterA.get_pv())
        self.ttempA2 = time.time() - self.start
        self.updateGUI(stamp="Temp A", data=float(self.tempA2))
        self.updateGUI(stamp="Time Temp A", data=float(self.ttempA2))
        print "ttempA: %.2f s\ttempA: %s C" % (self.ttempA2, self.tempA2)
    #end def

    #--------------------------------------------------------------------------
    def alpha(self,t1,t2):
        x = (t1+t2)/2 + 273.15

        # Seebeck correction for copper wires
        alpha = 25.28382238*x**0 - 1.769182633*x**1 + \
                0.05634706*x**2 - 0.001020289*x**3 + \
                1.17e-05*x**4 - 8.82e-08*x**5 + \
                4.49e-10*x**6 - 1.53e-12*x**7 + \
                3.31e-15*x**8 - 4.17e-18*x**9 + \
                2.31e-21*x**10

        print "seebeck for copper: %f"% (alpha)
        return alpha
    #end def

    #--------------------------------------------------------------------------
    def write_data_to_file(self):
        global myfile
        print('\nWrite data to file\n')
        time = (self.ttempA + self.ttempB + self.tVN + self.tVS + self.ttempA2 + self.ttempB2 + self.tVN2 + self.tVS2)/8
        ta = (self.tempA + self.tempA2)/2
        tb = (self.tempB + self.tempB2)/2
        avgt = (ta + tb)/2
        dt = ta-tb
        vn = (self.VN + self.VN2)/2
        vs = (self.VScalc + self.VScalc2)/2
        myfile.write('%.2f,%.4f,%.4f,%.4f,'%(time, self.x,self.y, self.B))
        myfile.write('%f,%f,' % (avgt, dt) )
        myfile.write('%.3f,%.3f\n' % (vn*10**6,vs*10**6))
    #end def

    #--------------------------------------------------------------------------
    def updateGUI(self, stamp, data):
        """
        Sends data to the GUI (main thread), for live updating while the process is running
        in another thread.
        """
        time.sleep(0.1)
        wx.CallAfter(pub.sendMessage, stamp, msg=data)
    #end def

    #--------------------------------------------------------------------------
    def save_files(self):
        ''' Function saving the files after the data acquisition loop has been
            exited.
        '''

        print('Save Files')

        global dataFile
        global finaldataFile
        global myfile
        global rawfile

        stop = time.time()
        end = datetime.now() # End time
        totalTime = stop - self.start # Elapsed Measurement Time (seconds)
        endStr = 'end time: %s \nelapsed measurement time: %s seconds \n \n' % (str(end), str(totalTime))

        myfile.close() # Close the file
        rawfile.close()

        myfile = open(dataFile, 'r') # Opens the file for Reading
        contents = myfile.readlines() # Reads the lines of the file into python set
        myfile.close()

        # Adds elapsed measurement time to the read file list
        contents.insert(1, endStr) # Specify which line and what value to insert
        # NOTE: First line is line 0

        # Writes the elapsed measurement time to the final file
        myfinalfile = open(finaldataFile,'w')
        contents = "".join(contents)
        myfinalfile.write(contents)
        myfinalfile.close()

        # Save the GUI plots
        global save_plots_ID
        save_plots_ID = 1
        self.updateGUI(stamp='Save_All', data='Save')
    #end def

#end class
###############################################################################

###############################################################################
class BoundControlBox(wx.Panel):
    """ A static box with a couple of radio buttons and a text
        box. Allows to switch between an automatic mode and a
        manual mode with an associated value.
    """
    #--------------------------------------------------------------------------
    def __init__(self, parent, ID, label, initval):
        wx.Panel.__init__(self, parent, ID)

        self.value = initval

        self.font = wx.Font(11, wx.DEFAULT, wx.NORMAL, wx.NORMAL)

        box = wx.StaticBox(self, -1, label)
        sizer = wx.StaticBoxSizer(box, wx.VERTICAL)

        self.radio_auto = wx.RadioButton(self, -1, label="Auto", style=wx.RB_GROUP)
        self.radio_manual = wx.RadioButton(self, -1, label="Manual")
        self.manual_text = wx.TextCtrl(self, -1, size=(30,-1), value=str(initval), style=wx.TE_PROCESS_ENTER)

        self.radio_auto.SetFont(self.font)
        self.radio_manual.SetFont(self.font)

        self.Bind(wx.EVT_UPDATE_UI, self.on_update_manual_text, self.manual_text)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.manual_text)

        manual_box = wx.BoxSizer(wx.HORIZONTAL)
        manual_box.Add(self.radio_manual, flag=wx.ALIGN_CENTER_VERTICAL)
        manual_box.Add(self.manual_text, flag=wx.ALIGN_CENTER_VERTICAL)

        sizer.Add(self.radio_auto, 0, wx.ALL, 10)
        sizer.Add(manual_box, 0, wx.ALL, 10)

        self.SetSizer(sizer)
        sizer.Fit(self)

    #end init

    #--------------------------------------------------------------------------
    def on_update_manual_text(self, event):
        self.manual_text.Enable(self.radio_manual.GetValue())

    #end def

    #--------------------------------------------------------------------------
    def on_text_enter(self, event):
        self.value = self.manual_text.GetValue()

    #end def

    #--------------------------------------------------------------------------
    def is_auto(self):
        return self.radio_auto.GetValue()

    #end def

    #--------------------------------------------------------------------------
    def manual_value(self):
        return self.value

    #end def

#end class
###############################################################################

###############################################################################
class UserPanel(wx.Panel):
    ''' User Input Panel '''

    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)

        global tolerance
        global stability_threshold
        global magrate
        global xdist
        global ydist
        global zdist


        self.celsius = u"\u2103"
        self.magrate = magrate
        self.xdist = xdist
        self.ydist = ydist
        self.zdist = zdist

        self.tolerance = tolerance
        self.stability_threshold = stability_threshold*60

        self.create_title("User Panel") # Title

        self.font2 = wx.Font(11, wx.DEFAULT, wx.NORMAL, wx.NORMAL)

        self.mag_rate_control()
        self.xdist_control()
        self.ydist_control()
        self.zdist_control()

        self.pid_tolerance_control()
        self.stability_threshold_control()

        self.maxMagCurrent_label()
        self.maxMagRate_label()

        self.linebreak1 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak2 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak3 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.linebreak4 = wx.StaticLine(self, pos=(-1,-1), size=(600,1), style=wx.LI_HORIZONTAL)

        self.run_stop() # Run and Stop buttons

        self.create_sizer() # Set Sizer for panel

        #pub.subscribe(self.post_process_data, "Post Process")
        pub.subscribe(self.enable_buttons, "Enable Buttons")

    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def run_stop(self):
        self.run_stopPanel = wx.Panel(self, -1)
        rs_sizer = wx.GridBagSizer(3, 3)


        self.btn_check = btn_check = wx.Button(self.run_stopPanel, label='check', style=0, size=(60,30)) # Run Button
        btn_check.SetBackgroundColour((0,0,255))
        caption_check = wx.StaticText(self.run_stopPanel, label='*check initial status')
        self.btn_run = btn_run = wx.Button(self.run_stopPanel, label='run', style=0, size=(60,30)) # Run Button
        btn_run.SetBackgroundColour((0,255,0))
        caption_run = wx.StaticText(self.run_stopPanel, label='*run measurement')
        self.btn_stop = btn_stop = wx.Button(self.run_stopPanel, label='stop', style=0, size=(60,30)) # Stop Button
        btn_stop.SetBackgroundColour((255,0,0))
        caption_stop = wx.StaticText(self.run_stopPanel, label = '*quit operation')

        btn_check.Bind(wx.EVT_BUTTON, self.check)
        btn_run.Bind(wx.EVT_BUTTON, self.run)
        btn_stop.Bind(wx.EVT_BUTTON, self.stop)

        controlPanel = wx.StaticText(self.run_stopPanel, label='Control Panel')
        controlPanel.SetFont(wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD))

        rs_sizer.Add(controlPanel,(0,0), span=(1,3),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(btn_check,(1,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_check,(2,0),flag=wx.ALIGN_CENTER_HORIZONTAL)

        rs_sizer.Add(btn_run,(1,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_run,(2,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(btn_stop,(1,2),flag=wx.ALIGN_CENTER_HORIZONTAL)
        rs_sizer.Add(caption_stop,(2,2),flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.run_stopPanel.SetSizer(rs_sizer)

        btn_stop.Disable()

    # end def

    #--------------------------------------------------------------------------
    def run(self, event):
        global k2182, k2700, ls643
        global dataFile
        global statusFile
        global fieldFile
        global finaldataFile
        global myfile
        global rawfile

        global abort_ID

        try:

            self.name_folder()

            if self.run_check == wx.ID_OK:

                file = dataFile # creates a data file
                myfile = open(dataFile, 'w') # opens file for writing/overwriting
                rawfile = open(statusFile,'w')
                begin = datetime.now() # Current date and time
                myfile.write('Start Time: ' + str(begin) + '\n')
                rawfile.write('Start Time: ' + str(begin) + '\n')

                data = 'avgtemp, deltatemp, Vn, Vs'
                headers = ( 'time, x, y, B,%s' % (data))

                myfile.write(headers)
                myfile.write('\n')

                rawheaders = 'time,x,y,tempA,setpointA,tempB,setpointB,magrate,magcurrent, B, Vn, Vs\n'
                rawfile.write(rawheaders)


                abort_ID = 0

                #start the threading process
                thread = ProcessThreadRun()

                self.btn_run.Disable()
                self.btn_check.Disable()
                self.btn_stop.Enable()

            #end if

        #end try

        except visa.VisaIOError:
            wx.MessageBox("Not all instruments are connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def check(self, event):
            InitialCheck()
    #end def

    #--------------------------------------------------------------------------
    def name_folder(self):
        question = wx.MessageDialog(None, 'The data files are saved into a folder upon ' + \
                    'completion. \nBy default, the folder will be named with a time stamp.\n\n' + \
                    'Would you like to name your own folder?', 'Question',
                    wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION)
        answer = question.ShowModal()

        if answer == wx.ID_YES:
            self.folder_name = wx.GetTextFromUser('Enter the name of your folder.\n' + \
                                                'Only type in a name, NOT a file path.')
            if self.folder_name == "":
                wx.MessageBox("Canceled")
            else:
                self.choose_dir()

        #end if

        else:
            date = str(datetime.now())
            self.folder_name = 'Nernst Effect Data %s.%s.%s' % (date[0:13], date[14:16], date[17:19])

            self.choose_dir()

        #end else

    #end def

    #--------------------------------------------------------------------------
    def choose_dir(self):
        found = False

        dlg = wx.DirDialog (None, "Choose the directory to save your files.", "",
                    wx.DD_DEFAULT_STYLE)

        self.run_check = dlg.ShowModal()

        if self.run_check == wx.ID_OK:
            global filePath
            filePath = dlg.GetPath()

            filePath = filePath + '/' + self.folder_name

            if not os.path.exists(filePath):
                os.makedirs(filePath)
                os.chdir(filePath)
            else:
                n = 1

                while found == False:
                    path = filePath + ' - ' + str(n)

                    if os.path.exists(path):
                        n = n + 1
                    else:
                        os.makedirs(path)
                        os.chdir(path)
                        n = 1
                        found = True

                #end while

            #end else

        #end if

        # Set the global path to the newly created path, if applicable.
        if found == True:
            filePath = path
        #end if
    #end def

    #--------------------------------------------------------------------------
    def stop(self, event):
        global abort_ID
        abort_ID = 1

        self.enable_buttons

    #end def

    #--------------------------------------------------------------------------
    def pid_tolerance_control(self):
        self.pid_tol_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_pid_tolerance = wx.StaticText(self,
                                            label="PID Tolerance ("+self.celsius+"):"
                                            )
        self.label_pid_tolerance.SetFont(self.font2)
        self.text_pid_tolerance = text_pid_tolerance = wx.StaticText(self.pid_tol_Panel, label=str(self.tolerance))
        text_pid_tolerance.SetFont(self.font2)
        self.edit_pid_tolerance = edit_pid_tolerance = wx.TextCtrl(self.pid_tol_Panel, size=(40, -1))
        self.btn_pid_tolerance = btn_pid_tolerance = wx.Button(self.pid_tol_Panel, label="save", size=(45, -1))
        text_guide = wx.StaticText(self.pid_tol_Panel, label="How close the temperature \nhas to be to the PID setpoint.")
        text_guide.SetFont(self.font2)
        btn_pid_tolerance.Bind(wx.EVT_BUTTON, self.save_pid_tolerance)

        hbox.Add((0, -1))
        #hbox.Add(self.label_pid_tolerance, 0 , wx.LEFT, 5)
        hbox.Add(text_pid_tolerance, 0, wx.LEFT, 5)
        hbox.Add(edit_pid_tolerance, 0, wx.LEFT, 5)
        hbox.Add(btn_pid_tolerance, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.pid_tol_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_pid_tolerance(self, e):
        global tolerance
        try:
            val = self.edit_pid_tolerance.GetValue()
            if float(val) > maxLimit:
                self.tolerance = str(maxLimit)
            else:
                self.tolerance = float(val)
            self.text_pid_tolerance.SetLabel(val)

            tolerance = self.tolerance

        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")
    #end def

    #--------------------------------------------------------------------------
    def stability_threshold_control(self):
        global stability_threshold
        self.stability_threshold_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_stability_threshold = wx.StaticText(self, label="Stability Threshold ("+self.celsius+"/min):")
        self.label_stability_threshold.SetFont(self.font2)
        self.text_stability_threshold = text_stability_threshold= wx.StaticText(self.stability_threshold_Panel, label=str(self.stability_threshold))
        text_stability_threshold.SetFont(self.font2)
        self.edit_stability_threshold = edit_stability_threshold = wx.TextCtrl(self.stability_threshold_Panel, size=(40, -1))
        self.btn_stability_threshold = btn_stability_threshold = wx.Button(self.stability_threshold_Panel, label="save", size=(45, -1))
        self.stability_text_guide = text_guide = wx.StaticText(self.stability_threshold_Panel, label=('The change in the PID must\n' +
                                                                      'be below this threshold range\nbefore a measurement will begin.'
                                                                      )
                                                            )
        text_guide.SetFont(self.font2)
        btn_stability_threshold.Bind(wx.EVT_BUTTON, self.save_stability_threshold)

        hbox.Add((0, -1))
        #hbox.Add(self.label_equil_threshold, 0 , wx.LEFT, 5)
        hbox.Add(text_stability_threshold, 0, wx.LEFT, 5)
        hbox.Add(edit_stability_threshold, 0, wx.LEFT, 5)
        hbox.Add(btn_stability_threshold, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.stability_threshold_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_stability_threshold(self, e):
        global stability_threshold

        try:
            val = self.edit_stability_threshold.GetValue()
            self.text_stability_threshold.SetLabel(val)
            self.stability_threshold = float(val)
            stability_threshold = self.stability_threshold/60

        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")
    #end def

    #--------------------------------------------------------------------------
    def mag_rate_control(self):
        global magrate
        self.mag_rate_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_mag_rate = wx.StaticText(self, label="Magnet Ramp Rate (A/s):")
        self.label_mag_rate.SetFont(self.font2)
        self.text_mag_rate = text_mag_rate = wx.StaticText(self.mag_rate_Panel, label=str(self.magrate))
        text_mag_rate.SetFont(self.font2)
        self.edit_mag_rate = edit_mag_rate = wx.TextCtrl(self.mag_rate_Panel, size=(40, -1))
        self.btn_mag_rate = btn_mag_rate = wx.Button(self.mag_rate_Panel, label="save", size=(45, -1))
        text_guide = wx.StaticText(self.mag_rate_Panel, label="The ramp rate of\nthe magnet.")
        text_guide.SetFont(self.font2)
        btn_mag_rate.Bind(wx.EVT_BUTTON, self.save_mag_rate)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_mag_rate, 0, wx.LEFT, 5)
        hbox.Add(edit_mag_rate, 0, wx.LEFT, 5)
        hbox.Add(btn_mag_rate, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.mag_rate_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_mag_rate(self, e):
        global magrate
        try:
            self.ls643 = ls643 # magPowerSupply

            val = self.edit_mag_rate.GetValue()

            if float(val) > maxMagRate:
                magrate = str(maxMagRate)

            self.text_mag_rate.SetLabel(val)

            magrate = float(val)
            self.magrate = magrate

            self.ls643.setRampRate(magrate)

        except ValueError:
            wx.MessageBox("Invalid input. Must be a number.", "Error")

        except visa.VisaIOError:
            wx.MessageBox("The SourceMeter is not connected!", "Error")
        #end except

    #end def

    #--------------------------------------------------------------------------
    def xdist_control(self):
       global xdist
       self.xdist_Panel = wx.Panel(self, -1)
       hbox = wx.BoxSizer(wx.HORIZONTAL)

       self.label_xdist = wx.StaticText(self, label="Sample Thickness - X (mm):")
       self.label_xdist.SetFont(self.font2)
       self.text_xdist = text_xdist = wx.StaticText(self.xdist_Panel, label=str(xdist))
       text_xdist.SetFont(self.font2)
       self.edit_xdist = edit_xdist = wx.TextCtrl(self.xdist_Panel, size=(40, -1))
       self.btn_xdist = btn_xdist = wx.Button(self.xdist_Panel, label="save", size=(45, -1))
       text_guide = wx.StaticText(self.xdist_Panel, label="The sample thickness\nalong the dT/dx direction.")
       text_guide.SetFont(self.font2)
       btn_xdist.Bind(wx.EVT_BUTTON, self.save_xdist)

       hbox.Add((0, -1))
       #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
       hbox.Add(text_xdist, 0, wx.LEFT, 5)
       hbox.Add(edit_xdist, 0, wx.LEFT, 5)
       hbox.Add(btn_xdist, 0, wx.LEFT, 5)
       hbox.Add(text_guide, 0, wx.LEFT, 5)

       self.xdist_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_xdist(self, e):
       global xdist
       val = self.edit_xdist.GetValue()
       self.text_xdist.SetLabel(val)
       xdist = val
       wx.CallAfter(pub.sendMessage, "X Distance", msg=xdist)

    #end def

    #--------------------------------------------------------------------------
    def ydist_control(self):
        global ydist
        self.ydist_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_ydist = wx.StaticText(self, label="Sample Thickness - Y (mm):")
        self.label_ydist.SetFont(self.font2)
        self.text_ydist = text_ydist = wx.StaticText(self.ydist_Panel, label=str(ydist))
        text_ydist.SetFont(self.font2)
        self.edit_ydist = edit_ydist = wx.TextCtrl(self.ydist_Panel, size=(40, -1))
        self.btn_ydist = btn_ydist = wx.Button(self.ydist_Panel, label="save", size=(45, -1))
        text_guide = wx.StaticText(self.ydist_Panel, label="The sample thickness\nalong the Nernst voltage direction.")
        text_guide.SetFont(self.font2)
        btn_ydist.Bind(wx.EVT_BUTTON, self.save_ydist)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_ydist, 0, wx.LEFT, 5)
        hbox.Add(edit_ydist, 0, wx.LEFT, 5)
        hbox.Add(btn_ydist, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.ydist_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_ydist(self, e):
        global ydist
        val = self.edit_ydist.GetValue()
        self.text_ydist.SetLabel(val)
        ydist = val
        wx.CallAfter(pub.sendMessage, "Y Distance", msg=ydist)

    #end def

    #--------------------------------------------------------------------------
    def zdist_control(self):
        global zdist
        self.zdist_Panel = wx.Panel(self, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.label_zdist = wx.StaticText(self, label="Magnet Pole Separation - Z (mm):")
        self.label_zdist.SetFont(self.font2)
        self.text_zdist = text_zdist = wx.StaticText(self.zdist_Panel, label=str(zdist))
        text_zdist.SetFont(self.font2)
        self.edit_zdist = edit_zdist = wx.TextCtrl(self.zdist_Panel, size=(40, -1))
        self.btn_zdist = btn_zdist = wx.Button(self.zdist_Panel, label="save", size=(45, -1))
        text_guide = wx.StaticText(self.zdist_Panel, label="The separation between\nthe magnet poles.")
        text_guide.SetFont(self.font2)
        btn_zdist.Bind(wx.EVT_BUTTON, self.save_zdist)

        hbox.Add((0, -1))
        #hbox.Add(self.label_current, 0 , wx.LEFT, 5)
        hbox.Add(text_zdist, 0, wx.LEFT, 5)
        hbox.Add(edit_zdist, 0, wx.LEFT, 5)
        hbox.Add(btn_zdist, 0, wx.LEFT, 5)
        hbox.Add(text_guide, 0, wx.LEFT, 5)

        self.zdist_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def save_zdist(self, e):
        global zdist
        val = self.edit_zdist.GetValue()
        self.text_zdist.SetLabel(val)
        zdist = val
        wx.CallAfter(pub.sendMessage, "Z Distance", msg=zdist)

    #end def

    #--------------------------------------------------------------------------
    def maxMagCurrent_label(self):
        global maxMagCurrent
        self.maxMagCurrent_Panel = wx.Panel(self, -1)
        maxMagCurrent_label = wx.StaticText(self.maxMagCurrent_Panel, label='Max Magnet Current:')
        maxMagCurrent_text = wx.StaticText(self.maxMagCurrent_Panel, label='%s A' % str(maxMagCurrent))
        maxMagCurrent_label.SetFont(self.font2)
        maxMagCurrent_text.SetFont(self.font2)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(maxMagCurrent_label, 0, wx.LEFT, 5)
        hbox.Add(maxMagCurrent_text, 0, wx.LEFT, 5)

        self.maxMagCurrent_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def maxMagRate_label(self):
        global maxMagRate
        self.maxMagRate_Panel = wx.Panel(self, -1)
        maxMagRate_label = wx.StaticText(self.maxMagRate_Panel, label='Max Magnet Ramp Rate:')
        maxMagRate_text = wx.StaticText(self.maxMagRate_Panel, label='%s A/s' % str(maxMagRate))
        maxMagRate_label.SetFont(self.font2)
        maxMagRate_text.SetFont(self.font2)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(maxMagRate_label, 0, wx.LEFT, 5)
        hbox.Add(maxMagRate_text, 0, wx.LEFT, 5)

        self.maxMagRate_Panel.SetSizer(hbox)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):

        sizer = wx.GridBagSizer(12,2)

        sizer.Add(self.titlePanel, (0, 1), span=(1,2), flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_mag_rate, (1, 1))
        sizer.Add(self.mag_rate_Panel, (1, 2))
        sizer.Add(self.label_pid_tolerance, (2, 1))
        sizer.Add(self.pid_tol_Panel, (2, 2))
        sizer.Add(self.label_stability_threshold, (3,1))
        sizer.Add(self.stability_threshold_Panel, (3, 2))

        sizer.Add(self.label_xdist, (4, 1))
        sizer.Add(self.xdist_Panel, (4, 2))
        sizer.Add(self.label_ydist, (5, 1))
        sizer.Add(self.ydist_Panel, (5, 2))
        sizer.Add(self.label_zdist, (6, 1))
        sizer.Add(self.zdist_Panel, (6, 2))

        sizer.Add(self.maxMagCurrent_Panel, (7, 1), span=(1,2))
        sizer.Add(self.maxMagRate_Panel, (8, 1), span=(1,2))

        sizer.Add(self.linebreak4, (9,1),span = (1,2))
        sizer.Add(self.run_stopPanel, (10,1),span = (1,2), flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)

    #end def

    def enable_buttons(self):
        self.btn_run.Enable()
        self.btn_check.Enable()
        self.btn_stop.Disable()

    #end def

#end class
###############################################################################

###############################################################################
class StatusPanel(wx.Panel):
    """
    Current Status of Measurements
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)

        self.celsius = u"\u2103"
        self.delta = u"\u0394"
        self.mu = u"\u00b5"

        self.ctime = str(datetime.now())[11:19]
        self.t='0:00:00'
        self.x=str(0.00)
        self.y=str(0.00)
        self.z=str(0.00)

        self.tA=str(30.0)
        self.tAset = str(40.0)
        self.stabilityA = '-'
        self.tB=str(30.0)
        self.tBset = str(40.0)
        self.stabilityB = '-'
        self.dT = str(float(self.tA)-float(self.tB))
        self.mi = str(0.00)
        self.mr = str(0.00)
        self.B = str(0.00)
        self.VN = str(0.00)
        self.VS = str(0.00)
        self.measurement = 'OFF'

        self.create_title("Status Panel")
        self.linebreak1 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))
        self.create_status()
        self.linebreak2 = wx.StaticLine(self, pos=(-1,-1), size=(300,1))

        self.linebreak3 = wx.StaticLine(self, pos=(-1,-1), size=(1,300), style=wx.LI_VERTICAL)

        # Updates from running program
        pub.subscribe(self.OnTime, "Time Nernst Voltage")
        pub.subscribe(self.OnTime, "Time Seebeck Voltage")
        pub.subscribe(self.OnTime, "Time Temp A")
        pub.subscribe(self.OnTime, "Time Temp B")
        pub.subscribe(self.OnTime, "Time Bfield")

        pub.subscribe(self.OnNernstVoltage, "Nernst Voltage")
        pub.subscribe(self.OnSeebeckVoltage, "Seebeck Voltage")
        pub.subscribe(self.OnTempA, "Temp A")
        pub.subscribe(self.OnTempAset, "Temp A SP")
        pub.subscribe(self.OnStabilityA, "Stability A")
        pub.subscribe(self.OnTempB, "Temp B")
        pub.subscribe(self.OnTempBset, "Temp B SP")
        pub.subscribe(self.OnStabilityB, "Stability B")

        pub.subscribe(self.OnMagCurrent, "Mag Current")
        pub.subscribe(self.OnMagRate, "Mag Rate")
        pub.subscribe(self.OnBfield, "Bfield")

        pub.subscribe(self.OnX, "X Distance")
        pub.subscribe(self.OnY, "Y Distance")
        pub.subscribe(self.OnZ, "Z Distance")

        pub.subscribe(self.OnMeasurement, "Measurement")


        pub.subscribe(self.OnNernstVoltage, "Nernst Voltage Status")
        pub.subscribe(self.OnSeebeckVoltage, "Seebeck Voltage Status")
        pub.subscribe(self.OnTempA, "Temp A Status")
        pub.subscribe(self.OnTempAset, "Temp A SP Status")
        pub.subscribe(self.OnTempB, "Temp B Status")
        pub.subscribe(self.OnTempBset, "Temp B SP Status")
        pub.subscribe(self.OnMagCurrent, "Mag Current Status")
        pub.subscribe(self.OnMagRate, "Mag Rate Status")
        pub.subscribe(self.OnBfield, "Bfield Status")

        #self.update_values()

        self.create_sizer()

    #end init

    #--------------------------------------------------------------------------
    def OnX(self, msg):
        self.x = '%.1f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnY(self, msg):
        self.y = '%.1f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnZ(self, msg):
        self.z= '%.1f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnNernstVoltage(self, msg):
        self.VN = '%.1f'%(float(msg)*10**6)
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnSeebeckVoltage(self, msg):
        self.VS = '%.1f'%(float(msg)*10**6)
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTempA(self, msg):
        self.tA = '%.1f'%(float(msg))
        self.dT = str(float(self.tA)-float(self.tB))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTempAset(self, msg):
        self.tAset = '%.1f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnStabilityA(self, msg):
        if msg != '-':
            self.stabilityA = '%.2f'%(float(msg))
        else:
            self.stabilityA = msg
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTempB(self, msg):
        self.tB = '%.1f'%(float(msg))
        self.dT = str(float(self.tA)-float(self.tB))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTempBset(self, msg):
        self.tBset = '%.1f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnStabilityB(self, msg):
        if msg != '-':
            self.stabilityB = '%.2f'%(float(msg))
        else:
            self.stabilityB = msg
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMagCurrent(self, msg):
        self.mi = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMagRate(self, msg):
        self.mr = '%.2f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnBfield(self, msg):
        self.B = '%.3f'%(float(msg))
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnMeasurement(self, msg):
        self.measurement = msg
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def OnTime(self, msg):
        time = int(float(msg))

        hours = str(time/3600)
        minutes = int(time%3600/60)
        if (minutes < 10):
            minutes = '0%i'%(minutes)
        else:
            minutes = '%i'%(minutes)
        seconds = int(time%60)
        if (seconds < 10):
            seconds = '0%i'%(seconds)
        else:
            seconds = '%i'%(seconds)

        self.t = '%s:%s:%s'%(hours,minutes,seconds)
        self.ctime = str(datetime.now())[11:19]
        self.update_values()
    #end def

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_status(self):
        self.label_ctime = wx.StaticText(self, label="current time:")
        self.label_ctime.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_t = wx.StaticText(self, label="run time:")
        self.label_t.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_x = wx.StaticText(self, label="thickness x (mm):")
        self.label_x.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_y = wx.StaticText(self, label="thickness y (mm):")
        self.label_y.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_z = wx.StaticText(self, label="pole separation z (mm):")
        self.label_z.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_tA = wx.StaticText(self, label="temp A ("+self.celsius+"):")
        self.label_tA.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_tAset = wx.StaticText(self, label="temp A setpoint("+self.celsius+"):")
        self.label_tAset.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_stabilityA = wx.StaticText(self, label="temp A stability ("+self.celsius+"/min):")
        self.label_stabilityA.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_tB = wx.StaticText(self, label="temp B ("+self.celsius+"):")
        self.label_tB.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_tBset = wx.StaticText(self, label="temp B setpoint("+self.celsius+"):")
        self.label_tBset.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_stabilityB = wx.StaticText(self, label="temp B stability ("+self.celsius+"/min):")
        self.label_stabilityB.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_dT = wx.StaticText(self, label=self.delta+"T ("+self.celsius+"):")
        self.label_dT.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_mi = wx.StaticText(self, label="magnet current (A):")
        self.label_mi.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_mr = wx.StaticText(self, label="magnet ramp rate (A/s):")
        self.label_mr.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_B = wx.StaticText(self, label="B field (T):")
        self.label_B.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_VN = wx.StaticText(self, label="nernst voltage ("+self.mu+"V):")
        self.label_VN.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_VS = wx.StaticText(self, label="seebeck voltage ("+self.mu+"V):")
        self.label_VS.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.label_measurement = wx.StaticText(self, label="measurement:")
        self.label_measurement.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))


        self.ctimecurrent = wx.StaticText(self, label=self.ctime)
        self.ctimecurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tcurrent = wx.StaticText(self, label=self.t)
        self.tcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.xcurrent = wx.StaticText(self, label=self.x)
        self.xcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.ycurrent = wx.StaticText(self, label=self.y)
        self.ycurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.zcurrent = wx.StaticText(self, label=self.z)
        self.zcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tAcurrent = wx.StaticText(self, label=self.tA)
        self.tAcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tAsetcurrent = wx.StaticText(self, label=self.tAset)
        self.tAsetcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.stabilityAcurrent = wx.StaticText(self, label=self.stabilityA)
        self.stabilityAcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tBcurrent = wx.StaticText(self, label=self.tB)
        self.tBcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.tBsetcurrent = wx.StaticText(self, label=self.tBset)
        self.tBsetcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.stabilityBcurrent = wx.StaticText(self, label=self.stabilityB)
        self.stabilityBcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.dTcurrent = wx.StaticText(self, label=self.dT)
        self.dTcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.micurrent = wx.StaticText(self, label=self.mi)
        self.micurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.mrcurrent = wx.StaticText(self, label=self.mr)
        self.mrcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.Bcurrent = wx.StaticText(self, label=self.B)
        self.Bcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.VNcurrent = wx.StaticText(self, label=self.VN)
        self.VNcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.VScurrent = wx.StaticText(self, label=self.VS)
        self.VScurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))
        self.measurementcurrent = wx.StaticText(self, label=self.measurement)
        self.measurementcurrent.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL))


    #end def

    #--------------------------------------------------------------------------
    def update_values(self):
        self.ctimecurrent.SetLabel(self.ctime)
        self.tcurrent.SetLabel(self.t)
        self.xcurrent.SetLabel(self.x)
        self.ycurrent.SetLabel(self.y)
        self.zcurrent.SetLabel(self.z)
        self.tAcurrent.SetLabel(self.tA)
        self.tAsetcurrent.SetLabel(self.tAset)
        self.stabilityAcurrent.SetLabel(self.stabilityA)
        self.tBcurrent.SetLabel(self.tB)
        self.tBsetcurrent.SetLabel(self.tBset)
        self.stabilityBcurrent.SetLabel(self.stabilityB)
        self.dTcurrent.SetLabel(self.dT)
        self.micurrent.SetLabel(self.mi)
        self.mrcurrent.SetLabel(self.mr)
        self.Bcurrent.SetLabel(self.B)
        self.VNcurrent.SetLabel(self.VN)
        self.VScurrent.SetLabel(self.VS)
        self.measurementcurrent.SetLabel(self.measurement)
    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(21,2)

        sizer.Add(self.titlePanel, (0, 0), span = (1,2), border=5, flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.linebreak1,(1,0), span = (1,2))

        sizer.Add(self.label_ctime, (2,0))
        sizer.Add(self.ctimecurrent, (2, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_t, (3,0))
        sizer.Add(self.tcurrent, (3, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_x, (4,0))
        sizer.Add(self.xcurrent, (4, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_y, (5,0))
        sizer.Add(self.ycurrent, (5, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_z, (6,0))
        sizer.Add(self.zcurrent, (6, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_tA, (7,0))
        sizer.Add(self.tAcurrent, (7,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_tAset, (8,0))
        sizer.Add(self.tAsetcurrent, (8,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_stabilityA, (9,0))
        sizer.Add(self.stabilityAcurrent, (9,1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_tB, (10,0))
        sizer.Add(self.tBcurrent, (10,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_tBset, (11,0))
        sizer.Add(self.tBsetcurrent, (11,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_stabilityB, (12,0))
        sizer.Add(self.stabilityBcurrent, (12,1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_dT, (13,0))
        sizer.Add(self.dTcurrent, (13,1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_mi, (14,0))
        sizer.Add(self.micurrent, (14, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_mr, (15,0))
        sizer.Add(self.mrcurrent, (15, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_B, (16,0))
        sizer.Add(self.Bcurrent, (16, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_VN, (17, 0))
        sizer.Add(self.VNcurrent, (17, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.label_VS, (18, 0))
        sizer.Add(self.VScurrent, (18, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.label_measurement, (19,0))
        sizer.Add(self.measurementcurrent, (19, 1),flag=wx.ALIGN_CENTER_HORIZONTAL)

        sizer.Add(self.linebreak2, (20,0), span = (1,2))

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class VoltagePanel(wx.Panel):
    """
    GUI Window for plotting voltage data.
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)
        global filePath

        global tVN_list
        global VN_list
        global tVS_list
        global VS_list

        self.create_title("Voltage Panel")
        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self, -1, self.figure)
        self.create_control_panel()
        self.create_sizer()

        pub.subscribe(self.OnNernstVoltage, "Nernst Voltage")
        pub.subscribe(self.OnNernstVTime, "Time Nernst Voltage")
        pub.subscribe(self.OnSeebeckVoltage, "Seebeck Voltage")
        pub.subscribe(self.OnSeebeckVTime, "Time Seebeck Voltage")

        # For saving the plots at the end of data acquisition:
        pub.subscribe(self.save_plot, "Save_All")

        self.animator = animation.FuncAnimation(self.figure, self.draw_plot, interval=500, blit=False)
    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_control_panel(self):

        self.xmin_control = BoundControlBox(self, -1, "t min", 0)
        self.xmax_control = BoundControlBox(self, -1, "t max", 2000)
        self.ymin_control = BoundControlBox(self, -1, "V min", -5)
        self.ymax_control = BoundControlBox(self, -1, "V max", 5)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.xmin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.xmax_control, border=5, flag=wx.ALL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.ymin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.ymax_control, border=5, flag=wx.ALL)
    #end def

    #--------------------------------------------------------------------------
    def OnNernstVoltage(self, msg):
        self.VN = float(msg)*10**6
        VN_list.append(self.VN)
        tVN_list.append(self.tVN)
    #end def

    #--------------------------------------------------------------------------
    def OnNernstVTime(self, msg):
        self.tVN = float(msg)
    #end def

    #--------------------------------------------------------------------------
    def OnSeebeckVoltage(self, msg):
        self.VS = float(msg)*10**6
        VS_list.append(self.VS)
        tVS_list.append(self.tVS)
    #end def

    #--------------------------------------------------------------------------
    def OnSeebeckVTime(self, msg):
        self.tVS = float(msg)
    #end def
    #--------------------------------------------------------------------------
    def init_plot(self):
        self.dpi = 100
        self.colorVN = 'g'
        self.colorVS = 'm'

        self.figure = Figure((5.5,3), dpi=self.dpi)
        self.subplot = self.figure.add_subplot(111)
        self.lineVN, = self.subplot.plot(tVN_list,VN_list, color=self.colorVN, linewidth=1)
        self.lineVS, = self.subplot.plot(tVS_list,VS_list, color=self.colorVS, linewidth=1)
        self.legend = self.figure.legend( (self.lineVN,self.lineVS), (r"$V_N$",r"$V_S$"), (0.15,0.75),fontsize=8)
        #self.subplot.text(0.05, .95, r'$X(f) = \mathcal{F}\{x(t)\}$', \
            #verticalalignment='top', transform = self.subplot.transAxes)
    #end def

    #--------------------------------------------------------------------------
    def draw_plot(self,i):
        self.subplot.clear()
        #self.subplot.set_title("voltage vs. time", fontsize=12)
        self.subplot.set_ylabel(r"voltage ($\mu V$)", fontsize = 8)
        self.subplot.set_xlabel("time (s)", fontsize = 8)

        # Adjustable scale:
        if self.xmax_control.is_auto():
            xmax = max(tVN_list+tVS_list)
        else:
            xmax = float(self.xmax_control.manual_value())
        if self.xmin_control.is_auto():
            xmin = 0
        else:
            xmin = float(self.xmin_control.manual_value())
        if self.ymin_control.is_auto():
            minV = min(VN_list+VS_list)
            ymin = minV - abs(minV)*0.3
        else:
            ymin = float(self.ymin_control.manual_value())
        if self.ymax_control.is_auto():
            maxV = max(VN_list+VS_list)
            ymax = maxV + abs(maxV)*0.3
        else:
            ymax = float(self.ymax_control.manual_value())


        self.subplot.set_xlim([xmin, xmax])
        self.subplot.set_ylim([ymin, ymax])

        pylab.setp(self.subplot.get_xticklabels(), fontsize=8)
        pylab.setp(self.subplot.get_yticklabels(), fontsize=8)

        self.lineVN, = self.subplot.plot(tVN_list,VN_list, color=self.colorVN, linewidth=1)
        self.lineVS, = self.subplot.plot(tVS_list,VS_list, color=self.colorVS, linewidth=1)

        return (self.lineVN,self.lineVS)
        #return (self.subplot.plot( thighV_list, highV_list, color=self.colorH, linewidth=1),
            #self.subplot.plot( tlowV_list, lowV_list, color=self.colorL, linewidth=1))

    #end def

    #--------------------------------------------------------------------------
    def save_plot(self, msg):
        path = filePath + "/Voltage_Plot.png"
        self.canvas.print_figure(path)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(3,1)
        sizer.Add(self.titlePanel, (0, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.canvas, ( 1,0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.hbox1, (2,0), flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class TemperaturePanel(wx.Panel):
    """
    GUI Window for plotting temperature data.
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)
        global filePath

        global ttempA_list
        global tempA_list
        global ttempB_list
        global tempB_list

        self.create_title("Temperature Panel")
        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self, -1, self.figure)
        self.create_control_panel()
        self.create_sizer()

        pub.subscribe(self.OnTimeTempA, "Time Temp A")
        pub.subscribe(self.OnTempA, "Temp A")
        pub.subscribe(self.OnTimeTempB, "Time Temp B")
        pub.subscribe(self.OnTempB, "Temp B")


        # For saving the plots at the end of data acquisition:
        pub.subscribe(self.save_plot, "Save_All")

        self.animator = animation.FuncAnimation(self.figure, self.draw_plot, interval=500, blit=False)
    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_control_panel(self):

        self.xmin_control = BoundControlBox(self, -1, "t min", 0)
        self.xmax_control = BoundControlBox(self, -1, "t max", 2000)
        self.ymin_control = BoundControlBox(self, -1, "T min", 0)
        self.ymax_control = BoundControlBox(self, -1, "T max", 70)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.xmin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.xmax_control, border=5, flag=wx.ALL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.ymin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.ymax_control, border=5, flag=wx.ALL)
    #end def

    #--------------------------------------------------------------------------
    def OnTimeTempA(self, msg):
        self.ttA = float(msg)

    #end def

    #--------------------------------------------------------------------------
    def OnTempA(self, msg):
        self.tA = float(msg)
        tempA_list.append(self.tA)
        ttempA_list.append(self.ttA)
    #end def

    #--------------------------------------------------------------------------
    def OnTimeTempB(self, msg):
        self.ttB = float(msg)

    #end def

    #--------------------------------------------------------------------------
    def OnTempB(self, msg):
        self.tB = float(msg)
        tempB_list.append(self.tB)
        ttempB_list.append(self.ttB)
    #end def

    #--------------------------------------------------------------------------
    def init_plot(self):
        self.dpi = 100
        self.colorTA = 'r'
        self.colorTB = 'b'

        self.figure = Figure((5.5,2), dpi=self.dpi)
        self.subplot = self.figure.add_subplot(111)

        self.lineTA, = self.subplot.plot(ttempA_list,tempA_list, color=self.colorTA, linewidth=1)
        self.lineTB, = self.subplot.plot(ttempB_list,tempB_list, color=self.colorTB, linewidth=1)

        self.legend = self.figure.legend( (self.lineTA, self.lineTB), (r"$T_A$",r"$T_B$"), (0.15,0.65),fontsize=8)
        #self.subplot.text(0.05, .95, r'$X(f) = \mathcal{F}\{x(t)\}$', \
            #verticalalignment='top', transform = self.subplot.transAxes)
    #end def

    #--------------------------------------------------------------------------
    def draw_plot(self,i):
        self.subplot.clear()
        #self.subplot.set_title("temperature vs. time", fontsize=12)
        self.subplot.set_ylabel(r"temperature ($\degree$C)", fontsize = 8)
        self.subplot.set_xlabel("time (s)", fontsize = 8)

        # Adjustable scale:
        if self.xmax_control.is_auto():
            xmax = max(ttempA_list+ttempB_list)
        else:
            xmax = float(self.xmax_control.manual_value())
        if self.xmin_control.is_auto():
            xmin = 0
        else:
            xmin = float(self.xmin_control.manual_value())
        if self.ymin_control.is_auto():
            ymin = 0
        else:
            ymin = float(self.ymin_control.manual_value())
        if self.ymax_control.is_auto():
            maxT = max(tempA_list+tempB_list)
            ymax = maxT + abs(maxT)*0.3
        else:
            ymax = float(self.ymax_control.manual_value())

        self.subplot.set_xlim([xmin, xmax])
        self.subplot.set_ylim([ymin, ymax])

        pylab.setp(self.subplot.get_xticklabels(), fontsize=8)
        pylab.setp(self.subplot.get_yticklabels(), fontsize=8)

        self.lineTA, = self.subplot.plot(ttempA_list,tempA_list, color=self.colorTA, linewidth=1)
        self.lineTB, = self.subplot.plot(ttempB_list,tempB_list, color=self.colorTB, linewidth=1)

        return (self.lineTA, self.lineTB)

    #end def

    #--------------------------------------------------------------------------
    def save_plot(self, msg):
        path = filePath + "/Temperature_Plot.png"
        self.canvas.print_figure(path)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(3,1)
        sizer.Add(self.titlePanel, (0, 0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.canvas, ( 1,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.hbox1, (2,0),flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class BfieldPanel(wx.Panel):
    """
    GUI Window for plotting temperature data.
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)
        global filePath

        global tB_list
        global B_list

        self.create_title("Magnetic Field Panel")
        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self, -1, self.figure)
        self.create_control_panel()
        self.create_sizer()

        pub.subscribe(self.OnTimeBfield, "Time Bfield")
        pub.subscribe(self.OnBfield, "Bfield")


        # For saving the plots at the end of data acquisition:
        pub.subscribe(self.save_plot, "Save_All")

        self.animator = animation.FuncAnimation(self.figure, self.draw_plot, interval=500, blit=False)
    #end init

    #--------------------------------------------------------------------------
    def create_title(self, name):
        self.titlePanel = wx.Panel(self, -1)
        title = wx.StaticText(self.titlePanel, label=name)
        font_title = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        title.SetFont(font_title)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add((0,-1))
        hbox.Add(title, 0, wx.LEFT, 5)

        self.titlePanel.SetSizer(hbox)
    #end def

    #--------------------------------------------------------------------------
    def create_control_panel(self):

        self.xmin_control = BoundControlBox(self, -1, "t min", 0)
        self.xmax_control = BoundControlBox(self, -1, "t max", 2000)
        self.ymin_control = BoundControlBox(self, -1, "B min", -3)
        self.ymax_control = BoundControlBox(self, -1, "B max", 3)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.xmin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.xmax_control, border=5, flag=wx.ALL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.ymin_control, border=5, flag=wx.ALL)
        self.hbox1.Add(self.ymax_control, border=5, flag=wx.ALL)
    #end def

    #--------------------------------------------------------------------------
    def OnTimeBfield(self, msg):
        self.tB = float(msg)

    #end def

    #--------------------------------------------------------------------------
    def OnBfield(self, msg):
        self.B = float(msg)
        B_list.append(self.B)
        tB_list.append(self.tB)
    #end def

    #--------------------------------------------------------------------------
    def init_plot(self):
        self.dpi = 100
        self.colorB = 'k'

        self.figure = Figure((5.5,2), dpi=self.dpi)
        self.subplot = self.figure.add_subplot(111)

        self.lineB, = self.subplot.plot(tB_list,B_list, color=self.colorB, linewidth=1)

        self.legend = self.figure.legend( (self.lineB,), (r"$B$",), (0.15,0.65),fontsize=8)
        #self.subplot.text(0.05, .95, r'$X(f) = \mathcal{F}\{x(t)\}$', \
            #verticalalignment='top', transform = self.subplot.transAxes)
    #end def

    #--------------------------------------------------------------------------
    def draw_plot(self,i):
        self.subplot.clear()
        #self.subplot.set_title("temperature vs. time", fontsize=12)
        self.subplot.set_ylabel(r"magnetic field ($T$)", fontsize = 8)
        self.subplot.set_xlabel("time (s)", fontsize = 8)

        # Adjustable scale:
        if self.xmax_control.is_auto():
            xmax = max(tB_list)
        else:
            xmax = float(self.xmax_control.manual_value())
        if self.xmin_control.is_auto():
            xmin = 0
        else:
            xmin = float(self.xmin_control.manual_value())
        if self.ymin_control.is_auto():
            minB = min(B_list)
            ymin = minB - abs(minB)*0.3
        else:
            ymin = float(self.ymin_control.manual_value())
        if self.ymax_control.is_auto():
            maxB = max(B_list)
            ymax = maxB + abs(maxB)*0.3
        else:
            ymax = float(self.ymax_control.manual_value())

        self.subplot.set_xlim([xmin, xmax])
        self.subplot.set_ylim([ymin, ymax])

        pylab.setp(self.subplot.get_xticklabels(), fontsize=8)
        pylab.setp(self.subplot.get_yticklabels(), fontsize=8)

        self.lineB, = self.subplot.plot(tB_list,B_list, color=self.colorB, linewidth=1)

        return (self.lineB)

    #end def

    #--------------------------------------------------------------------------
    def save_plot(self, msg):
        path = filePath + "/Magnetic_Field_Plot.png"
        self.canvas.print_figure(path)

    #end def

    #--------------------------------------------------------------------------
    def create_sizer(self):
        sizer = wx.GridBagSizer(3,1)
        sizer.Add(self.titlePanel, (0, 0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.canvas, ( 1,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.hbox1, (2,0),flag=wx.ALIGN_CENTER_HORIZONTAL)

        self.SetSizer(sizer)
    #end def

#end class
###############################################################################

###############################################################################
class Frame(wx.Frame):
    """
    Main frame window in which GUI resides
    """
    #--------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        wx.Frame.__init__(self, *args, **kwargs)
        self.init_UI()
        self.create_statusbar()
        self.create_menu()

        pub.subscribe(self.update_statusbar, "Status Bar")

    #end init

    #--------------------------------------------------------------------------
    def init_UI(self):
        self.SetBackgroundColour('#E0EBEB')
        self.userpanel = UserPanel(self, size=wx.DefaultSize)
        self.statuspanel = StatusPanel(self,size=wx.DefaultSize)
        self.voltagepanel = VoltagePanel(self, size=wx.DefaultSize)
        self.temperaturepanel = TemperaturePanel(self, size=wx.DefaultSize)
        self.bfieldpanel = BfieldPanel(self, size=wx.DefaultSize)

        self.statuspanel.SetBackgroundColour('#ededed')

        sizer = wx.GridBagSizer(2, 3)
        sizer.Add(self.userpanel, (0,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.bfieldpanel, (1,0),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.voltagepanel, (0,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.temperaturepanel, (1,1),flag=wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.statuspanel, (0,2),flag=wx.ALIGN_CENTER_HORIZONTAL, span = (2,1))


        sizer.Fit(self)

        self.SetSizer(sizer)
        self.SetTitle('Room Temp Nernst GUI')
        self.Centre()
    #end def

    #--------------------------------------------------------------------------
    def create_menu(self):
        # Menu Bar with File, Quit
        menubar = wx.MenuBar()
        fileMenu = wx.Menu()
        qmi = wx.MenuItem(fileMenu, APP_EXIT, '&Quit\tCtrl+Q')
        #qmi.SetBitmap(wx.Bitmap('exit.png'))
        fileMenu.AppendItem(qmi)

        self.Bind(wx.EVT_MENU, self.onQuit, id=APP_EXIT)

        menubar.Append(fileMenu, 'File')
        self.SetMenuBar(menubar)
    #end def

    #--------------------------------------------------------------------------
    def onQuit(self, e):
        global abort_ID

        abort_ID=1
        self.Destroy()
        self.Close()

        sys.stdout.close()
        sys.stderr.close()
    #end def

    #--------------------------------------------------------------------------
    def create_statusbar(self):

        self.statusbar = ESB.EnhancedStatusBar(self, -1)

        self.statusbar.SetSize((-1, 23))

        self.statusbar.SetFieldsCount(8)
        self.SetStatusBar(self.statusbar)

        self.space_between = 10

        ### Create Widgets for the statusbar:
        # Status:
        self.status_text = wx.StaticText(self.statusbar, -1, "Ready")
        self.width0 = 105

        # Placer 1:
        placer1 = wx.StaticText(self.statusbar, -1, " ")

        # Title:
        #measurement_text = wx.StaticText(self.statusbar, -1, "Measurement Indicators:")
        #boldFont = wx.Font(9, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        #measurement_text.SetFont(boldFont)
        #self.width1 = measurement_text.GetRect().width + self.space_between

        # PID Tolerance:
        pidTol_text = wx.StaticText(self.statusbar, -1, "Within PID Tolerance:")
        self.width2 = pidTol_text.GetRect().width + self.space_between

        self.indicator_tol = wx.StaticText(self.statusbar, -1, "-")
        self.width3 = 25

        # Stability Threshold:
        stableThresh_text = wx.StaticText(self.statusbar, -1, "Within Stability Threshold:")
        self.width4 = stableThresh_text.GetRect().width + 5

        self.indicator_stable = wx.StaticText(self.statusbar, -1, "-")
        self.width5 = self.width3

        # Placer 2:
        placer2 = wx.StaticText(self.statusbar, -1, " ")

        # Version:
        version_label = wx.StaticText(self.statusbar, -1, "Version: %s" % version)
        self.width8 = version_label.GetRect().width + self.space_between

        # Set widths of each piece of the status bar:
        self.statusbar.SetStatusWidths([self.width0, 50, self.width2, self.width3, self.width4, self.width5, -1, self.width8])

        ### Add the widgets to the status bar:
        # Status:
        self.statusbar.AddWidget(self.status_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # Placer 1:
        self.statusbar.AddWidget(placer1)

        # Title:
        #self.statusbar.AddWidget(measurement_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # PID Tolerance:
        self.statusbar.AddWidget(pidTol_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)
        self.statusbar.AddWidget(self.indicator_tol, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

        # Stability Threshold:
        self.statusbar.AddWidget(stableThresh_text, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)
        self.statusbar.AddWidget(self.indicator_stable, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)


        # Placer 2
        self.statusbar.AddWidget(placer2)

        # Version:
        self.statusbar.AddWidget(version_label, ESB.ESB_ALIGN_CENTER_HORIZONTAL, ESB.ESB_ALIGN_CENTER_VERTICAL)

    #end def

    #--------------------------------------------------------------------------
    def update_statusbar(self, msg):
        string = msg

        # Status:
        if string == 'Running' or string == 'Finished, Ready' or string == 'Exception Occurred' or string=='Checking':
            self.status_text.SetLabel(string)
            self.status_text.SetBackgroundColour(wx.NullColour)

            if string == 'Exception Occurred':
                self.status_text.SetBackgroundColour("RED")
            #end if

        #end if

        else:
            tol = string[0]
            stable = string[1]

            # PID Tolerance indicator:
            self.indicator_tol.SetLabel(tol)
            if tol == 'OK':
                self.indicator_tol.SetBackgroundColour("GREEN")
            #end if
            else:
                self.indicator_tol.SetBackgroundColour("RED")
            #end else

            # Stability Threshold indicator:
            self.indicator_stable.SetLabel(stable)
            if stable == 'OK':
                self.indicator_stable.SetBackgroundColour("GREEN")
            #end if
            else:
                self.indicator_stable.SetBackgroundColour("RED")
            #end else
        #end else

    #end def

#end class
###############################################################################

###############################################################################
class App(wx.App):
    """
    App for initializing program
    """
    #--------------------------------------------------------------------------
    def OnInit(self):
        self.frame = Frame(parent=None, title="Room Temp Nernst GUI", size=(1280,1280))
        self.frame.Show()

        setup = Setup()
        return True
    #end init

#end class
###############################################################################

#==============================================================================
if __name__=='__main__':
    app = App()
    app.MainLoop()

#end if
