import base64
import cv2
import json
import sys
import threading
import queue

class Bridge:

	def thread(self):

		for line in sys.stdin:
			self.queue.put(line)

	def __init__(self):

		self.manualControl = False
		self.manualState   = [False, False, False, False, False, False, False, False]
		self.abort = False

		self.queue = queue.Queue()

		self.thread = threading.Thread(target=self.thread)
		self.thread.daemon = True
		self.thread.start()

	def getInput(self):

		try: string = self.queue.get(timeout=0)
		except queue.Empty: return

		info = json.loads(string)

		try:

			self.manualControl = info["manualControl"];
			print(F"manualControl: {self.manualControl}", file=sys.stderr)

		except KeyError: pass

		try:

			self.manualState = info["manualState"];
			print(F"manualState: {self.manualState}", file=sys.stderr)

		except KeyError: pass

		try:

			self.abort = info["abort"];
			print(F"abort: {self.abort}", file=sys.stderr)

		except KeyError: pass

	def setImage(self, img):

		# encode in jpeg to save bandwidth
		data = cv2.imencode(".jpg", img)[1]

		# json format
		encoded = json.dumps({"frame": base64.b64encode(data).decode("ascii")})

		# send to stderr
		print(encoded)
