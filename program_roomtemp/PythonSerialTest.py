from time import sleep
import serial
ser = serial.Serial("/dev/cu.SLAB_USBtoUART",57600,serial.SEVENBITS,serial.PARITY_ODD, serial.STOPBITS_ONE, None,False,False,None,False,None)# Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish

def main():
     #print ser.readline()
     #news = 0
     a = 1
     #while True:
     sleep(.1)
     news = 'RDGV? \r\n'
     ser.write(news)
     a= ser.readline()
     print a
     #dub =  ser.readline()
     #print dub**2
     ser.close()
main()
