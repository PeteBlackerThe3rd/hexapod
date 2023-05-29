from inputs import get_gamepad, UnpluggedError


class GamePad:

  def __init__(self):

    self.connected = False
    self.state = self.get_state()

  def get_state(self):
    try:
      events = get_gamepad()
      axes = {}
      for event in events:
        if event.ev_type == 'Absolute':
          if event.code == 'ABS_X':
            axes['LeftX'] = event.state
          elif event.code == 'ABS_Y':
            axes['LeftY'] = event.state
          elif event.code == 'ABS_RX':
            axes['RightX'] = event.state
          elif event.code == 'ABS_RY':
            axes['RightY'] = event.state
      self.connected = True
      return axes

    except UnpluggedError:
      self.connected = False
      return {'LeftX': 0, 'LeftY': 0, 'RightX': 0, 'RightY': 0}
