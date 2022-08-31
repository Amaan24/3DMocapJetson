import serial
import time
import keyboard

#* HTTPS://WWW.DROPBOX.COM/SH/H4STAGLDKM8JNJ0/AAD_9L8YGL0YVCEZRNSALR1KA/SPECIFICATIONS%20AND%20INSTRUCTIONS?DL=0&PREVIEW=MOTOR+DRIVER+INSTRUCTIONS+R210303.PDF&SUBFOLDER_NAV_TRACKING=1 *#

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=400000, timeout=0.500)
					
ser.flushInput()
ser.flushOutput()


CR = 0x0D #Carriage Return, ends all commands to and from controller

READ_FW = 0x76 #Read firmware version, b'v\x05\x03\x01\x00\x00\r' =
READ_SN = 0x79 #Read serial number, b'y\x05P\x00\x006\x92\r' = 0x79\0x05\0x50\0x00\0x00\0x36\0x92

###Motor Control
WRITE_MS = 0x63 #Write motor setup, easier to just use provided windows software
READ_MS = 0x67 #Read motor setup, easier to just use provided windows software

##Move Stepper Motors
#| Command ID | Motor ID | Steps (MSB) | Steps (LSB) | Start/Stop | Speed (MSB) | Speed (LSB) | CR |
#Steps/Speed = (256*MSB)+LSB 
#Zoom max steps = 3256, speed 600-1000
#Focus max steps = 8467, speed 600-1000
#Iris max steps = 75, speed 100-200 
#Returns 0x74 0x00 0x0D = t\x00\r if successful 
#Command IDs
ROT_TELE = 0x66
ROT_WIDE = 0x62
ROT_PI = 0x73

#Motor IDs
ID_F = 0x01 
ID_Z = 0x02
ID_I = 0x03
ID_IR = 0x04

STRT = 0x01
STOP = 0x00

#Set speed of 800pps for zoom and focus
SPD_MSB = 0x03
SPD_LSB = 0x20
#Set speed of 200 pps for iris
SPD_MSB_I = 0x00
SPD_LSB_I = 0xC8

#Set step size for zoom, focus and iris
STEP_MSB_Z = 0x00 
STEP_LSB_Z = 0x64
STEP_MSB_F = 0x00 
STEP_LSB_F = 0xC8
STEP_MSB_I = 0x00 
STEP_LSB_I = 0x0F

zoom_pos = 0
focus_pos = 0

def zoom_tele():
	ser.write(bytearray([ROT_TELE, ID_Z, STEP_MSB_Z, STEP_LSB_Z, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def zoom_wide():
	ser.write(bytearray([ROT_WIDE, ID_Z, STEP_MSB_Z, STEP_LSB_Z, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def zoom_step(steps_msb, steps_lsb):
	ser.write(bytearray([ROT_PI, ID_Z, steps_msb, steps_lsb, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def focus_tele():
	ser.write(bytearray([ROT_TELE, ID_F, STEP_MSB_F, STEP_LSB_F, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def focus_wide():
	ser.write(bytearray([ROT_WIDE, ID_F, STEP_MSB_F, STEP_LSB_F, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def focus_step(steps_msb, steps_lsb):
	ser.write(bytearray([ROT_PI, ID_F, steps_msb, steps_lsb, STRT, SPD_MSB, SPD_LSB,  CR]))
	raw = ser.readline()
	print(raw)
	return

def iris_open():
	ser.write(bytearray([ROT_TELE, ID_I, STEP_MSB_I, STEP_LSB_I, STRT, SPD_MSB_I, SPD_LSB_I,  CR]))
	raw = ser.readline()
	print(raw)
	return

def iris_close():
	ser.write(bytearray([ROT_WIDE, ID_I, STEP_MSB_I, STEP_LSB_I, STRT, SPD_MSB_I, SPD_LSB_I,  CR]))
	raw = ser.readline()
	print(raw)
	return

def main():
	while True:
		input_key = keyboard.read_key() 
		if  input_key == "up":
			zoom_wide()
#			time.sleep(0.2)
		elif input_key == "down":
			zoom_tele()
			print("You pressed down")
#			time.sleep(0.2)
		elif input_key == "left":
			print("You pressed left")
			focus_tele()
#			time.sleep(0.2)
		elif input_key == "right":
			focus_wide()
			print("You pressed right")
#			time.sleep(0.2)
		elif input_key == "+":
			iris_open()
			print("You pressed +")
#			time.sleep(0.2)
		elif input_key == "-":
			iris_close()
			print("You pressed -")
#			time.sleep(0.2)
		elif input_key == "z":
			zoom_step(0x00, 0x00)
			print("You pressed z")
		elif input_key == "f":
			focus_step(0x00, 0x00)
			print("You pressed f")
#			time.sleep(0.2)
		elif input_key == "-":
			iris_close()
			print("You pressed -")
#			time.sleep(0.2)
		elif input_key == "esc":
			break

if __name__ == "__main__":
    main()

