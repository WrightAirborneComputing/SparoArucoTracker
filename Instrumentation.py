import sys
from datetime import datetime

class Inst:

	OFF = 0
	NO_CR = 1
	CR = 2
	PLING = 3
	DOT = 4
	QUEST = 5

	def __init__(self): 
		# Create timer start
		self.timerStart = 0
		self.timerMilliseconds = 0
		self.loopStart = datetime.now()
	# def 

	def StartTick(self,milliseconds):
		self.timerStart = datetime.now()
		self.timerMilliseconds = milliseconds
	# def

	def Tick(self):
		delay = datetime.now() - self.timerStart
		delayMilliseconds = (delay.seconds * 1000) + (delay.microseconds / 1000)
		if(delayMilliseconds>=self.timerMilliseconds):
			self.StartTick(self.timerMilliseconds)
			return True
		else:
			return False
		#end if
	# def 

	def Print(self,format,text):
		if(format==Inst.NO_CR):
			print(text, end='')
		elif(format==Inst.CR):
			print(text)
		elif(format==Inst.DOT):
			print(".", end='')
		elif(format==Inst.PLING):
			print("!", end='')
		elif(format==Inst.QUEST):
			print("?", end='')
		# end if
		sys.stdout.flush()
	# def

	def Log(self,text):
		timeStr = datetime.now().strftime("%Y/%m/%d %H:%M:%S:%f")
		self.Print(Inst.CR,timeStr + ": " + text)

		file = open(self._logFileName, "a+")
		file.write(timeStr + ": " + text + "\r\n")
		file.close()
	#end def

	def LogImage(self,image,suffix):
		timeStr = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
		logFileName = self._logDirectoryName + "/" + timeStr + "_" + suffix + ".jpg"
		cv2.imwrite(logFileName, image, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
	#end def

# def class


