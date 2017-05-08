import RPi.GPIO as GPIO
import smbus
import picamera
import time

global bilderPfad
bilderPfad = "/home/pi/Desktop/KAMERA/"

LUX = 30
SENSOR_PIN = 21
RELAIS_PIN = 26
cam = picamera.PiCamera()

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(RELAIS_PIN, GPIO.HIGH)
GPIO.setup(RELAIS_PIN, GPIO.OUT)
GPIO.setup(RELAIS_PIN, GPIO.HIGH)

DEVICE     = 0x23 # Default device I2C address
POWER_DOWN = 0x00 # No active state
POWER_ON   = 0x01 # Power on
RESET      = 0x07 # Reset data register value

CONTINUOUS_LOW_RES_MODE = 0x13
CONTINUOUS_HIGH_RES_MODE_1 = 0x10
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
ONE_TIME_HIGH_RES_MODE_1 = 0x20
ONE_TIME_HIGH_RES_MODE_2 = 0x21
ONE_TIME_LOW_RES_MODE = 0x23

bus = smbus.SMBus(1)

def convertToNumber(data):
  return ((data[1] + (256 * data[0])) / 1.2)

def readLight(addr=DEVICE):
  data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
  return convertToNumber(data)

def main():

  while True:
    print "Light Level : " + str(readLight()) + " lx"
    time.sleep(1)

    if readLight() < LUX:
      print "unter " +  str(LUX)
      if GPIO.input(SENSOR_PIN) == GPIO.HIGH:
        print "bewegung!!"
        GPIO.setup(RELAIS_PIN, GPIO.LOW)
        cam.start_preview()
        cam.resolution = (1920, 1080)
        cam.vflip = True
        time.sleep(2)
        timestr = time.strftime("%Y"".""%m"".""%d-%H"":""%M"":""%S")
        print "Bildaufnahme unter Dateiname bild_",timestr, "gespeichert."
        cam.capture(bilderPfad + 'bild_{}.jpg'.format(timestr))
        #cam.stop_preview()
        #cam.close()
        time.sleep(8)
        GPIO.setup(RELAIS_PIN, GPIO.HIGH)
      else: print "keine Bewegung"
    else: print "ueber " + str(LUX)
  
if __name__=="__main__":
   main()
