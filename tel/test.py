import spidev
import RPi.GPIO as GPIO
import time

NRESET = 17
BUSY = 22
IRQ = 27
ANT_SW = 23

GET_STATUS = 0xC0
READ_REG = 0x19
SET_SLEEP = 0x84

spi = spidev.SpiDev()

def setup_gpio():
  print('-- GPIO Pins --')
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(ANT_SW, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(BUSY, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.setup(IRQ, GPIO.IN)
  GPIO.setup(NRESET, GPIO.OUT, initial=GPIO.HIGH)

  print(f'ANT_SW: {ANT_SW} (OUT)')
  print(f'BUSY:   {BUSY}   (IN)')
  print(f'IRQ:    {IRQ}    (IN)')
  print(f'NRESET: {NRESET} (OUT)')
  print()

def setup_spi():
  print('SPI: /dev/spidev0.0')
  spi.open(0, 0)
  spi.max_speed_hz = 1_000_000
  print(f'SPI Max Speed: {spi.max_speed_hz} Hz')
  print()

def wait_busy():
  print('Waiting for BUSY = 0')

  while GPIO.input(BUSY) == GPIO.HIGH:
    pass

  print('Done waiting for BUSY')

def hardware_reset():
  print('Resetting...')
  print('NRESET = 0')
  GPIO.output(NRESET, GPIO.LOW)
  time.sleep(0.1)

  print('NRESET = 1')
  GPIO.output(NRESET, GPIO.HIGH)
  wait_busy()

def transfer(data):
  print(f'transfer: {data}')
  wait_busy()
  response = spi.xfer2(data)
  print(f'response: {response}')
  wait_busy()
  print('transfer done')
  print()

  return response

setup_gpio()
setup_spi()
hardware_reset()

status = transfer([GET_STATUS, 0x00])
print(f'GetStatus -> {status}')

GPIO.cleanup()
spi.close()
