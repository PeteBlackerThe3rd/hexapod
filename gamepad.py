# from inputs import get_gamepad, GamePad as ipGamePad, UnpluggedError, devices
import threading
import copy
import time
from pyjoystick.sdl2 import Key, Joystick, run_event_loop, get_mapping_name


class GamePad:

  def __init__(self):

    self.connected = False
    self._state = {'leftx': 0.0, 'lefty': 0.0, 'rightx': 0.0, 'righty': 0.0}
    self._state_lock = threading.Lock()
    self._listener_thread = threading.Thread(target=self.listener_thread_entrypoint)
    self._listener_thread.start()

  def get_state(self):
    self._state_lock.acquire()
    state_copy = copy.copy(self._state)
    self._state_lock.release()
    return state_copy

  def added(self, joystick):
    self.connected = True

  def removed(self, joystick):
    self.connected = False

  def event_received(self, key):
    key_name = get_mapping_name(key.joystick, key)
    if key_name in self._state.keys():
      self._state_lock.acquire()
      self._state[key_name] = float(key.value)
      self._state_lock.release()
      # print("%s updated to %f" % (key_name, float(key.value)))

  def listener_thread_entrypoint(self):
    run_event_loop(self.added, self.removed, self.event_received)


if __name__ == "__main__":

  pad = GamePad()
  while True:
    print(pad.get_state())
    time.sleep(0.03)

