#! /usr/bin/python
# -*- coding: utf-8 -*-
import os
import matplotlib

from matplotlib.figure import Figure
from matplotlib.pyplot import gcf, setp
import matplotlib.animation as animation # For plotting
import pylab
import numpy as np
import matplotlib.pyplot as plt
import visa # pyvisa, essential for communicating with the Keithley
from threading import Thread # For threading the processes going on behind the GUI
import time
from datetime import datetime # for getting the current date and time

current = .01 # (A) Current that is sourced by the k2400

# Global placers for instruments
k2700 = ''
k2400 = ''
k2182 = ''
ls643 = ''

#ResourceManager for visa instrument control
ResourceManager = visa.ResourceManager()

###############################################################################
class Keithley_2700:
    ''' Used for the matrix card operations. '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)
        
        self.openAllChannels()
    #end init
        
    #--------------------------------------------------------------------------
    def closeChannels(self, channels):
        self.ctrl.write(":ROUT:MULT:CLOS (@ %s)" %channels)
    #end def
        
    #--------------------------------------------------------------------------
    def openChannels(self, channels):
        self.ctrl.write(":ROUT:MULT:OPEN (@ %s)" %channels)
    #end def
    
    #--------------------------------------------------------------------------
    def openAllChannels(self):
        self.ctrl.write("ROUTe:OPEN:ALL")
    #end def
    
    #--------------------------------------------------------------------------
    def get_closedChannels(self):
        return self.ctrl.query(":ROUT:MULT:CLOS?")
    #end def

#end class
###############################################################################

###############################################################################
class Keithley_2400:
    ''' SourceMeter '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)
        
        self.ctrl.write(":ROUT:TERM REAR") # Use the rear output terminals
        self.current_mode()
        self.set_current_range(10.5*10**(-3)) # Default
        self.set_current(float(current))
    #end init
        
    #--------------------------------------------------------------------------
    def turn_source_on(self):
        self.ctrl.write(":OUTPut:STATe ON")
    #end def
        
    #--------------------------------------------------------------------------
    def turn_source_off(self):
        self.ctrl.write(":OUTPut:STATe OFF")
    #end def
        
    #--------------------------------------------------------------------------
    def query_state(self):
        state = self.ctrl.query(":OUTPut:STATe?")
        
        if state == '1':
            amplitude = self.ctrl.query(":SOURce:CURRent:LEVel:IMMediate:AMPLitude?") + ' Amps'
            
            if amplitude == '0.000000E+00 Amps':
                amplitude = self.ctrl.query(":SOURce:VOLTage:LEVel:IMMediate:AMPLitude?") + ' Volts'
            
            return 'state: %s, amplitude: %s ' % (state, amplitude)
            
        else:
            return 'state: %s' % state
            
    #end def
            
    #--------------------------------------------------------------------------
    def current_mode(self):
        self.ctrl.write(":SOURce:FUNCtion:MODE CURRent")
        self.ctrl.write(":SOURce:CURRent:MODE FIXed") # Fixed current mode
    #end def
        
    #--------------------------------------------------------------------------
    def set_current(self, current):
        self.change_current_range(current)
        #time.sleep(5)
        self.ctrl.write(":SOURce:CURRent:LEVel:IMMediate:AMPLitude %f" % current)
    #end def
        
    #--------------------------------------------------------------------------
    def change_current_range(self, current):
        #self.write(":SOURce:CURRent:LEVel:IMMediate:AMPLitude 0")
        if current > 0:
            if current > 105*10**(-3):
                    self.set_current_range(1.05)
            else:
                if current > 10.5*10**(-3):
                    self.set_current_range(105*10**(-3))
                else:
                    if current > 1.05*10**(-3):
                        self.set_current_range(10.5*10**(-3))
                    else:
                        if current > 105*10**(-6):
                            self.set_current_range(1.05*10**(-3))
                        else:
                            if current > 10.5*10**(-6):
                                self.set_current_range(105*10**(-6))
                            else:
                                if current > 1.05*10**(-6):
                                    self.set_current_range(10.5*10**(-6))
                                else:
                                    self.set_current_range(1.05*10**(-6))
        
        elif current < 0:
            if current < -105*10**(-3):
                    self.set_current_range(-1.05)
            else:
                if current < -10.5*10**(-3):
                    self.set_current_range(-105*10**(-3))
                else:
                    if current < -1.05*10**(-3):
                        self.set_current_range(-10.5*10**(-3))
                    else:
                        if current < -105*10**(-6):
                            self.set_current_range(-1.05*10**(-3))
                        else:
                            if current < -10.5*10**(-6):
                                self.set_current_range(-105*10**(-6))
                            else:
                                if current < -1.05*10**(-6):
                                    self.set_current_range(-10.5*10**(-6))
                                else:
                                    self.set_current_range(-1.05*10**(-6))
            
        else:
            self.set_current_range(1.05*10**(-6))
            
    #end def
                                    
    #--------------------------------------------------------------------------
    def set_current_range(self, current):
        self.ctrl.write(":SOURce:CURRent:RANGe %f" % current)
        
    #end def
      
    #--------------------------------------------------------------------------
    def voltage_mode(self):
        self.ctrl.write(":SOURce:FUNCtion:MODE VOLTage")
        self.ctrl.write(":SOURce:VOLTage:MODE FIXed") # Fixed voltage mode
    #end def
        
    #--------------------------------------------------------------------------
    def set_voltage(self, voltage):
        self.ctrl.write(":SOURce:VOLTage:LEVel:IMMediate:AMPLitude %f" % voltage)
    #end def

#end class
###############################################################################

###############################################################################
class Keithley_2182:
    ''' NanoVoltMeter '''
    #--------------------------------------------------------------------------
    def __init__(self, instr):
        self.ctrl = ResourceManager.open_resource(instr)
        
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
        return str(data)[0:15] # Fetches Reading

    #end def

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
        global k2400
        global k2182
        
        # Define Keithley instrument ports:
        self.k2700 = k2700 = Keithley_2700('GPIB0::2::INSTR') # MultiMeter for Matrix Card operation
        self.k2400 = k2400 = Keithley_2400('GPIB0::3::INSTR') # SourceMeter
        self.k2182 = k2182 = Keithley_2182('GPIB0::4::INSTR') # NanoVoltMeter

#end class
###############################################################################

###############################################################################
class CheckIV:
    #--------------------------------------------------------------------------
    def __init__(self):
        global k2700
        global k2400
        global k2182
        
        global current
        
        
        self.k2700 = k2700
        self.k2400 = k2400
        self.k2182 = k2182
        
        self.delay = .7
        self.current = .01
        
        #self.I = {}
        #self.V = {}
        #self.V_fit = {}
        self.Data = {}
        
        # short the matrix card
        self.k2700.closeChannels('117, 125, 126, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        time.sleep(self.delay)
        
        self.measure_contacts()
        
        self.create_plot()
    #end init
    #--------------------------------------------------------------------------
    
    #--------------------------------------------------------------------------
    def measure_contacts(self):

        # r_12
        print('measure r_12')
        self.k2700.openChannels('126, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_12 = self.checkIV('A','B')
        self.k2700.closeChannels('126, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        print "r12: %f Ohm" % (self.r_12)
    
        time.sleep(self.delay)
    
        # r_13
        print('measure r_13')
        self.k2700.closeChannels('119')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_13 = self.checkIV('A','C')
        self.k2700.closeChannels('117, 125, 126, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('119')
        print(self.k2700.get_closedChannels())
        print "r13: %f Ohm" % (self.r_13)
    
        time.sleep(self.delay)
    
        # r_24
        print('measure r_24')
        self.k2700.closeChannels('120')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 126, 127, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_24 = self.checkIV('B','D')
        self.k2700.closeChannels('117, 125, 126, 127, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('120')
        print(self.k2700.get_closedChannels())
        print "r24: %f Ohm" % (self.r_24)
    
        time.sleep(self.delay)
    
        # r_34
        print('measure r_34')
        self.k2700.closeChannels('118')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('117, 125, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.r_34 = self.checkIV('C','D')
        self.k2700.closeChannels('117, 125, 127, 128, 129, 130')
        print(self.k2700.get_closedChannels())
        self.k2700.openChannels('118')
        print(self.k2700.get_closedChannels())
        print "r34: %f Ohm" % (self.r_34)
        
    #end def
    #--------------------------------------------------------------------------
    
    #--------------------------------------------------------------------------
    def checkIV(self,p1,p2):
        print('check IV')
        n = 2
        I = [1000*self.current*(x)/n for x in range(-n,n+1)]
        V = []
        
        for i in I:
            self.k2400.turn_source_on()
            self.k2400.set_current(float(i)/1000)
            time.sleep(self.delay) 
            v = float( self.k2182.fetch() )
            print 'i: %f\nv: %f'%(i,v)
            V.append(v)
            time.sleep(self.delay)
        #end for
        
        
        fit = self.polyfit(V,I,1)
        
        self.Data[p1+p2] = fit
        
        self.Data[p1+p2]['current'] = I
        self.Data[p1+p2]['voltage'] = V
        
        r = (fit['polynomial'][0])**-1
         
        return r
    #end def
    #--------------------------------------------------------------------------
    
    #--------------------------------------------------------------------------
    def polyfit(self, x, y, degree):
        '''
        Returns the polynomial fit for x and y of degree degree along with the
        r^2 and the temperature, all in dictionary form.
        '''
        results = {}
    
        coeffs = np.polyfit(x, y, degree)
    
        # Polynomial Coefficients
        results['polynomial'] = coeffs.tolist()
        
        # Calculate coefficient of determination (r-squared):
        p = np.poly1d(coeffs)
        # fitted values:
        yhat = p(x)                      # or [p(z) for z in x]
        # mean of values:
        ybar = np.sum(y)/len(y)          # or sum(y)/len(y)
        # regression sum of squares:
        ssreg = np.sum((yhat-ybar)**2)   # or sum([ (yihat - ybar)**2 for yihat in yhat])
        # total sum of squares:
        sstot = np.sum((y - ybar)**2)    # or sum([ (yi - ybar)**2 for yi in y])
        results['r-squared'] = ssreg / sstot
    
        return results
    
    #end def
    
    #--------------------------------------------------------------------------
    def create_plot(self):
        plt.figure(num='IV Curves', figsize=(12,9),dpi=100)
        fitData = {}
        sp = 221
        for key in self.Data.keys():
            fitData[key] = {}
            i = np.poly1d(self.Data[key]['polynomial'])
            v = np.linspace(min(self.Data[key]['voltage']), max(self.Data[key]['voltage']), 500)
            
            fitData[key]['current'] = i(v)
            fitData[key]['voltage'] = v
            fitData[key]['equation'] = 'I = %.4f*(V) + %.4f' % (self.Data[key]['polynomial'][0], self.Data[key]['polynomial'][1])
            
            plt.subplot(sp)
            sp = sp + 1                
            plt.plot(self.Data[key]['voltage'],self.Data[key]['current'],'r.',fitData[key]['voltage'],fitData[key]['current'],'b--')
            plt.xlabel("V (V)")
            plt.ylabel("I (mA)")
            plt.title('IV curve - '+key)
            plt.legend(('i-v data','fit: '+fitData[key]['equation']),loc=4,fontsize=10)
            #plt.axis([ , , , ])
            plt.grid(True)
        #end for
        
        
        #fig.savefig('%s.png' % (plot_folder + title) , dpi=dpi)
        #plt.savefig('%s.png' % ('IV Curves') )
        plt.show()
        
    #end def
    
#end class
###############################################################################

#==============================================================================
            
def main():
    sp = Setup()
    checkIV = CheckIV()
#end def
    
if __name__ == '__main__':
    main()
#end if



