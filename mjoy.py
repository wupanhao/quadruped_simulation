#!coding:utf-8
import struct
import time

BUTTON = 129
AXIS = 130

EVENT_BUTTON = 1
EVENT_AXIS = 2

class MyJoy(object):
	"""docstring for MyJoy"""
	def __init__(self,callback=None):
		super(MyJoy, self).__init__()
		self.active = True
		self.callback = callback
		self.axes = {0:0,1:0,2:0,3:0,4:0,5:0,6:0,7:0}
		self.buttons = {0:0,1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0}
		self.infile_path = "/dev/input/js0"
	def start_listen_loop(self):
		EVENT_SIZE = struct.calcsize("LhBB")
		file = open(self.infile_path, "rb")
		event = file.read(EVENT_SIZE)
		while event:
			# print(struct.unpack("LhBB", event))
			(tv_msec,  value, event_type, number) = struct.unpack("LhBB", event)
			# print(tv_msec,  value, event_type, number)
			if event_type == EVENT_BUTTON or event_type == BUTTON:
				self.buttons[number] = value
			elif event_type == EVENT_AXIS or event_type == AXIS:
				self.axes[number] = value
			print(self.getState())
			if self.callback is not None:
				self.callback()
			event = file.read(EVENT_SIZE)

	def start_open_loop(self):
		while self.active:
			try:
				self.start_listen_loop()
			except Exception as e:
				print(e)
				print('Read %s Error, retry after 2 seconds',self.infile_path)
				time.sleep(2)
	def getState(self):
		state = {'Axes':self.axes,'Buttons':self.buttons}
		return state

if __name__ == '__main__':
	from car import CarDriver3
	import threading
	car = CarDriver3()
	joy = MyJoy(callback=None)
	reader = threading.Thread(target=joy.start_open_loop)
	reader.daemon = True
	reader.start()	
	def setCarSpeed():
		if joy.axes.has_key(1) and joy.axes.has_key(3):
			speed = joy.axes[1]/32767.0
			steer = joy.axes[3]/32767.0
			car.setWheelsSpeed(speed*100,-steer*90)	
	# joy = MyJoy(callback=setCarSpeed)
	while True:
		setCarSpeed()
		time.sleep(0.01)
	joy.active = False

