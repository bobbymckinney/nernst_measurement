import serial
import time

###############################################################################
class Lakeshore_643(serial.Serial):
    '''Electromagnet Power Supply'''
    #--------------------------------------------------------------------------
    def __init__(self, port):
        serial.Serial.__init__(self, port, 57600,serial.SEVENBITS,serial.PARITY_ODD, serial.STOPBITS_ONE, None,False,False,None,False,None)
        
        self.term = '\r\n'
        self.delay = 2
        
        # clear error
        #self.clearError()
        
        # clear and reset power supply
        #self.clearInstrument()
        #self.resetInstrument()
        
        #set current to 0
        #self.setCurrent(0)
        
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
            self.write('LIMIT' + str(current) + ',' + str(rate) + self.term)
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

def main():
     ls643 = Lakeshore_643("/dev/cu.SLAB_USBtoUART")
     
     print ls643.getCurrent()
     print ls643.getVoltage()
     
     
"""     error = ls643.errorTest
     
     print error
     
     print '\n'
     
     current = ls643.getCurrent()
     
     print current"""
     
     
     
if __name__=='__main__':
    main()