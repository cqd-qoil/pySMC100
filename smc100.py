#!/usr/bin/env python
import serial
import time

# never wait for more than this e.g. during wait_state
MAX_WAIT_TIME_SEC = 12

# time to wait after sending a command. This number has been arrived at by
# trial and error
COMMAND_WAIT_TIME_SEC = 0.06

# States from page 65 of the manual
STATE_NOT_REFERENCED_FROM_RESET = '0A'
STATE_NOT_REFERENCED_FROM_CONFIGURATION = '0C'
STATE_READY_FROM_HOMING = '32'
STATE_READY_FROM_MOVING = '33'

STATE_DISABLE_FROM_READY = '3C'
STATE_DISABLE_FROM_MOVING = '3D'
STATE_DISABLE_FROM_JOGGING = '3E'

class SMC100ReadTimeOutException(Exception):
  def __init__(self):
    super(SMC100ReadTimeOutException, self).__init__('Read timed out')

class SMC100WaitTimedOutException(Exception):
  def __init__(self):
    super(SMC100WaitTimedOutException, self).__init__('Wait timed out')

class SMC100DisabledStateException(Exception):
  def __init__(self):
    super(SMC100DisabledStateException, self).__init__('Disabled state encountered')

class SMC100(object):
  """
  Class to interface with Newport's SMC100 controller.

  The SMC100 accepts commands in the form of:

    <ID><command><arguments><CR><LF>

  Reply, if any, will be in the form

    <ID><command><result><CR><LF>

  There is minimal support for manually setting stage parameter as Newport's ESP stages can
  supply the SMC100 with the correct configuration parameters.

  Some effort is made to take up backlash, but this should not be trusted too much.
  """

  _port = None
  _smcID = None

  _silent = True

  def __init__(self, smcID, port, backlash_compensation=True, silent=True):
    """
    If backlash_compensation is False, no backlash compensation will be done.

    If silent is False, then additional output will be emitted to aid in debugging.
    """

    super(SMC100, self).__init__()

    assert smcID is not None
    assert port is not None

    self._silent = silent

    print 'Connecting to SMC100 on %s'%(port)

    self._port = serial.Serial(
        port = port,
        baudrate = 57600,
        bytesize = 8,
        stopbits = 1,
        parity = 'N',
        timeout = 0.050)

    self._smcID = str(smcID)

    self.sendcmd('RS')

    self.wait_state(STATE_NOT_REFERENCED_FROM_RESET, ignore_disabled_states=True)

    stage = self.sendcmd('ID', '?', True)
    print 'Found stage', stage

    # enter config mode
    self.sendcmd('PW', 1)

    # load stage parameters
    self.sendcmd('ZX', 1)

    # enable stage ID check
    self.sendcmd('ZX', 2)

    # exit configuration mode
    self.sendcmd('PW', 0)

    # wait for us to get back into NOT REFERENCED state
    self.wait_state(STATE_NOT_REFERENCED_FROM_CONFIGURATION)

    # home so we are no longer NOT REFERENCED
    self.home()

    # wait for the controller to be ready
    self.wait_state(STATE_READY_FROM_HOMING)

  def home(self):
    self.sendcmd('OR')

  def get_status(self):
    """
    Executes TS? and returns the the error code as integer and state as string
    as specified on pages 64 - 65 of the manual.
    """

    resp = self.sendcmd('TS', '?', True)
    errors = int(resp[0:4], 16)
    state = resp[4:]

    assert len(state) == 2

    return errors, state

  def move_absolute_mm(self, position_mm, waitStop=True):
    """
    Move the stage to the given absolute position (mm).

    If wait is True, then this method returns when the stage is at the
    specified position and motion has stopped.
    """
    self.sendcmd('PA', position_mm)
    if waitStop:
      self.wait_state(STATE_READY_FROM_MOVING)

  def wait_state(self, targetstate, ignore_disabled_states=False):
    """
    Waits for the controller to enter the specified target state. Controller state is determined
    via the TS command.

    If ignore_disabled_states is True, disable states are ignored. The normal behaviour when
    encountering a disabled state when not looking for one is for an exception to be raised.

    Note that this method will ignore read timeouts and keep trying until the controller responds.
    Because of this it can be used to determine when the controller is ready again after a
    command like PW0 which can take up to 10 seconds to execute.

    If any disable state is encountered, the method will raise an error, UNLESS you were waiting
    for that state. This is because if we wait for READY_FROM_MOVING, and the stage gets stuck
    we transition into DISABLE_FROM_MOVING and then STAY THERE FOREVER.
    """
    starttime = time.time()
    done = False
    while not done:
      waittime = time.time() - starttime
      if waittime > MAX_WAIT_TIME_SEC:
        raise SMC100WaitTimedOutException()

      try:
        state = self.get_status()[1]
        if targetstate == state:
          return
        elif not ignore_disabled_states:
          disabledstates = [
              STATE_DISABLE_FROM_READY,
              STATE_DISABLE_FROM_JOGGING,
              STATE_READY_FROM_MOVING]
          if state in disabledstates:
            raise SMC100DisabledStateException()

      except SMC100ReadTimeOutException:
        self._emit('Read timed out, retrying in 1 second')
        time.sleep(1)
        continue

  def sendcmd(self, command, argument=None, expect_response=False):
    """
    Send the specified command along with the argument, if any. The response is checked to ensure
    it has the correct prefix, and is returned WITHOUT the prefix.

    It is important that for GET commands, e.g. 1ID?, the ? is specified as an ARGUMENT, not as
    part of the command. Doing so will result in assertion failure.

    If expect_response is True, a response is expected from the controller which will be verified
    and returned without the prefix.
    """
    assert command[-1] != '?'

    if argument is None:
      argument = ''

    prefix = self._smcID + command
    tosend = prefix + str(argument)

    self._port.flushInput()
    self._port.write(tosend)
    self._port.write('\r\n')
    self._port.flush()

    time.sleep(COMMAND_WAIT_TIME_SEC)

    if not self._silent:
      self._emit('sent', tosend)

    if expect_response:
      response = self._readline()
      assert response.startswith(prefix)
      return response[len(prefix):]
    else:
      return None

  def _readline(self):
    """
    Returns a line, that is reads until \r\n.

    OK, so you are probably wondering why I wrote this. Why not just use
    self._port.readline()?

    I am glad you asked.

    With python < 2.6, pySerial uses serial.FileLike, that provides a readline
    that accepts the max number of chars to read, and the end of line
    character.

    With python >= 2.6, pySerial uses io.RawIOBase, whose readline only
    accepts the max number of chars to read. io.RawIOBase does support the
    idea of a end of line character, but it is an attribute on the instance,
    which makes sense... except pySerial doesn't pass the newline= keyword
    argument along to the underlying class, and so you can't actually change
    it.
    """
    done = False
    line = str()
    #print 'reading line',
    while not done:
      c = self._port.read()
      # ignore \r since it is part of the line terminator
      if len(c) == 0:
        raise SMC100ReadTimeOutException()
      elif c == '\r':
        continue
      elif c == '\n':
        done = True
      else:
        line += c

    self._emit('read', line)

    return line

  def _emit(self, *args):
    if len(args) == 1:
      prefix = ''
      message = args[0]
    else:
      prefix = ' ' + args[0]
      message = args[1]

    if not self._silent:
      print '[SMC100' + prefix + '] ' + message

  def close(self):
    if self._port:
      self._port.close()
      self._port = None

  def __del__(self):
    self.close()
# Tests #####################################################################
def test_init():
  smc100 = SMC100(1, '/dev/ttyS0', silent=False)

  # make sure there are no errors
  assert smc100.get_status()[0] == 0

  smc100.move_absolute_mm(10)
  smc100.move_absolute_mm(0)

  del smc100

