
import time
from colorama import Fore, Style

from logger import Logger, Level
from config_loader import ConfigLoader
from pixel import Pixel
from status import Status
from motor_controller import MotorController
from payload import Payload

class Driver:
    def __init__(self):
        self._log = Logger('motor_test', level=Level.INFO)
        self._log.info('reading configuration…')
        _config = ConfigLoader.configure('config.yaml')
        _pixel = Pixel(_config, pixel_count=8, brightness=0.1)
        self._status = Status(_pixel)
        self._log.info('setting up motor controller…')
        self._motor_controller = MotorController(config=_config, status=self._status, level=Level.INFO)
        self._motor_controller.enable()
        self._log.info('ready.')

    def run(self):
        time.sleep(5)
        try:
            self._log.info('accelerating…')
            for speed in range(0, 101, 5):
                payload = Payload('GO', speed, speed, speed, speed)
                self._log.info(Fore.GREEN + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.2)
            time.sleep(2.5)
            self._log.info('decelerating…')
            for speed in range(100, -1, -5):
                payload = Payload('GO', speed, speed, speed, speed)
                self._log.info(Fore.YELLOW + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.2)

            # now rotate
            self._log.info('accelerating…')
            for speed in range(0, 101, 5):
                payload = Payload('RO', speed, speed, speed, speed)
                self._log.info(Fore.GREEN + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.09)
            time.sleep(0.45)
            self._log.info('decelerating…')
            for speed in range(100, -1, -5):
                payload = Payload('RO', speed, speed, speed, speed)
                self._log.info(Fore.YELLOW + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.09)

            self._log.info('accelerating…')
            for speed in range(0, 101, 5):
                payload = Payload('GO', speed, speed, speed, speed)
                self._log.info(Fore.GREEN + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.2)
            time.sleep(2.5)
            self._log.info('decelerating…')
            for speed in range(100, -1, -5):
                payload = Payload('GO', speed, speed, speed, speed)
                self._log.info(Fore.YELLOW + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.2)

            # now rotate CCW
            self._log.info('accelerating…')
            for speed in range(0, 101, 5):
                payload = Payload('RC', speed, speed, speed, speed)
                self._log.info(Fore.GREEN + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.09)
            time.sleep(0.45)
            self._log.info('decelerating…')
            for speed in range(100, -1, -5):
                payload = Payload('RC', speed, speed, speed, speed)
                self._log.info(Fore.YELLOW + 'payload: {}'.format(payload))
                self._motor_controller.go(payload)
                time.sleep(0.09)

            time.sleep(1)
        except Exception as e:
            self._log.error('{} error raised: {}'.format(type(e), e))
        finally:
            self._log.info('complete.')

    def motor_id(self):
        '''
        Runs each motor in succession in order to identify them.
        '''
        try:
            # Payload(self, code, pfwd, sfwd, paft, saft)
            speed = 35
            # PFWD ...............................
            self._log.info('M0: PFWD.')
            payload = Payload('GO', speed, 0, 0, 0)
            self._motor_controller.go(payload)
            time.sleep(5)
            self._motor_controller.stop()
            # SFWD ...............................
            self._log.info('M1: SFWD.')
            payload = Payload('GO', 0, speed, 0, 0)
            self._motor_controller.go(payload)
            time.sleep(5)
            self._motor_controller.stop()
            # PAFT ...............................
            self._log.info('M2: PAFT.')
            payload = Payload('GO', 0, 0, speed, 0)
            self._motor_controller.go(payload)
            time.sleep(5)
            self._motor_controller.stop()
            # SAFT ...............................
            self._log.info('M3: SAFT.')
            payload = Payload('GO', 0, 0, 0, speed)
            self._motor_controller.go(payload)
            time.sleep(5)
            self._motor_controller.stop()
        except Exception as e:
            self._log.error('{} error raised: {}'.format(type(e), e))
        finally:
            self._log.info('complete.')

    def close(self):
        if self._motor_controller:
            self._motor_controller.close()
        if self._status:
            self._status.off()
        self._log.info('closed.')

# for REPL usage or testing
def exec():
    driver = None
    try:
        driver = Driver()
#       driver.motor_id()
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
        if driver:
            driver.close()

if __name__ == "__main__":
    driver = None
    try:
        driver = Driver()
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
        if driver:
            driver.close()

#EOF
