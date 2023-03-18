import loralib
import time
import serial
import smbus
from bmp280 import BMP280
from picamera2 import Picamera2
from libcamera import controls
from smbus2 import SMBus
import adafruit_gps

#Accelerometr settings

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

   
c=0
f = open('/home/wb/logs.txt','a')

#BMP settings
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)

#LoRa settings
loralib.init(0,434000000,11)

#Camera Settings
cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (2000, 2000)})
cam.configure(config)
cam.set_controls({"AfMode":  controls.AfModeEnum.Continuous})
cam.start()
cam.capture_file("/home/wb/zdjecia/{}.jpg".format(c/10))


#GPS settings
ser = serial.Serial(
        port='/dev/tty1',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=10)
gps = adafruit_gps.GPS(ser, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b'PMTK220,1000')



while 1:
	#BMP280 data handling 
	temperature = bmp280.get_temperature()
	pressure = bmp280.get_pressure()
	
	#GPS data handling
	gps.update()
	
	#Accelometr
	
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)

	Ax = acc_z/16384.0
	Ay = acc_y/16384.0
	Az = acc_x/16384.0

	#Sending data
	send ="{};{:05.2f};{:05.2f};{};{};{};{:05.2f};{:05.2f};{:05.2f};".format(c,pressure,temperature,gps.longitude,gps.latitude,gps.altitude_m,Ax,Ay,Az)
	loralib.send(bytes(send,"utf-8",'ignore'))
	
	#saving data logs
	print(send )
	f.write(send+"\n")
	f.flush()
	
	#Making photo
	if c %10==0:
		cam.capture_file("/home/wb/zdjecia/{}.jpg".format(c/10))
		
	c=c+1
	time.sleep(1)
	
	
    
    

