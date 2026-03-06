from time import sleep
from typing import Optional

try:
	import RPi.GPIO as GPIO
	_GPIO_AVAILABLE = True
except Exception:
	# テスト環境やデスクトップでの静的解析向けのフォールバック
	_GPIO_AVAILABLE = False

	class _FakePWM:
		def __init__(self, pin, freq):
			self.pin = pin
			self.freq = freq
			self.duty = 0

		def start(self, duty):
			self.duty = duty
			print(f"[FAKE PWM start] pin={self.pin} freq={self.freq} duty={duty}")

		def ChangeDutyCycle(self, duty):
			self.duty = duty
			print(f"[FAKE PWM change] pin={self.pin} duty={duty}")

		def stop(self):
			print(f"[FAKE PWM stop] pin={self.pin}")

	class _FakeGPIO:
		BCM = 'BCM'
		OUT = 'OUT'
		LOW = 0
		HIGH = 1

		def setmode(self, m):
			print(f"[FAKE GPIO] setmode {m}")

		def setup(self, pin, mode, initial=None):
			print(f"[FAKE GPIO] setup pin={pin} mode={mode} initial={initial}")

		def output(self, pin, val):
			print(f"[FAKE GPIO] output pin={pin} val={val}")

		def PWM(self, pin, freq):
			return _FakePWM(pin, freq)

		def cleanup(self):
			print("[FAKE GPIO] cleanup")

	GPIO = _FakeGPIO()


class TB6612Driver:
	"""TB6612FNG を使って 2 つのモーター (A, B) を制御するドライバクラス。

	コンストラクタに各INピンとPWMピン、STBYピンを渡します。
	PWM は RPi.GPIO.PWM を使って速度制御します。
	"""

	def __init__(
		self,
		ain1: int,
		ain2: int,
		pwma: int,
		bin1: int,
		bin2: int,
		pwmb: int,
		stby: Optional[int] = None,
		pwm_freq: int = 1000,
	):
		self.ain1 = ain1
		self.ain2 = ain2
		self.pwma = pwma
		self.bin1 = bin1
		self.bin2 = bin2
		self.pwmb = pwmb
		self.stby = stby
		self.pwm_freq = pwm_freq

		GPIO.setmode(GPIO.BCM)

		# 出力ピンを初期化
		for p in (ain1, ain2, bin1, bin2):
			GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

		if stby is not None:
			GPIO.setup(stby, GPIO.OUT, initial=GPIO.LOW)

		# PWM 初期化
		self._pwma = GPIO.PWM(pwma, pwm_freq)
		self._pwmb = GPIO.PWM(pwmb, pwm_freq)
		self._pwma.start(0)
		self._pwmb.start(0)

		# スタンバイ解除
		if stby is not None:
			GPIO.output(stby, GPIO.HIGH)

	def _apply(self, in1, in2, pwm: float, pwm_obj):
		# pwm: 0..100
		GPIO.output(in1[0], in1[1])
		GPIO.output(in2[0], in2[1])
		pwm_obj.ChangeDutyCycle(max(0, min(100, pwm)))

	def set_motor(self, motor: str, speed: float) -> None:
		"""モーター 'A' か 'B' を speed (-100..100) で回す。

		speed > 0: 前進
		speed < 0: 後進
		speed == 0: ブレーキ(両INを同じにしてPWM=0)
		"""
		speed = max(-100.0, min(100.0, float(speed)))

		if motor.upper() == 'A':
			in1 = (self.ain1, GPIO.HIGH if speed > 0 else GPIO.LOW)
			in2 = (self.ain2, GPIO.LOW if speed > 0 else GPIO.HIGH)
			pwm_val = abs(speed)
			self._apply(in1, in2, pwm_val, self._pwma)
		elif motor.upper() == 'B':
			in1 = (self.bin1, GPIO.HIGH if speed > 0 else GPIO.LOW)
			in2 = (self.bin2, GPIO.LOW if speed > 0 else GPIO.HIGH)
			pwm_val = abs(speed)
			self._apply(in1, in2, pwm_val, self._pwmb)
		else:
			raise ValueError("motor must be 'A' or 'B'")

	def stop(self):
		"""両モーター停止、PWM 0 にしてスタンバイにする（もしSTBYがあるならLOW）。"""
		self._pwma.ChangeDutyCycle(0)
		self._pwmb.ChangeDutyCycle(0)
		GPIO.output(self.ain1, GPIO.LOW)
		GPIO.output(self.ain2, GPIO.LOW)
		GPIO.output(self.bin1, GPIO.LOW)
		GPIO.output(self.bin2, GPIO.LOW)
		if self.stby is not None:
			GPIO.output(self.stby, GPIO.LOW)

	def cleanup(self):
		try:
			self.stop()
			self._pwma.stop()
			self._pwmb.stop()
		finally:
			GPIO.cleanup()


if __name__ == '__main__':
	# ピン設定は環境に合わせて変更してください
	AIN1 = 17
	AIN2 = 27
	PWMA = 18
	BIN1 = 22
	BIN2 = 23
	PWMB = 24
	STBY = 25

	print("TB6612FNG ドライバテストを開始します。Ctrl+C で停止")
	driver = TB6612Driver(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY)

	try:
		print("モーターA/B を前進 50% で 2 秒")
		driver.set_motor('A', 50)
		driver.set_motor('B', 50)
		sleep(2)

		print("モーターA を後進 40%、モーターB を前進 80% で 2 秒")
		driver.set_motor('A', -40)
		driver.set_motor('B', 80)
		sleep(2)

		print("停止")
		driver.set_motor('A', 0)
		driver.set_motor('B', 0)
		sleep(1)
	except KeyboardInterrupt:
		print("中断検出: 停止します")
	finally:
		driver.cleanup()

