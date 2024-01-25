#!/usr/bin/env python
import serial
import time

controller_state_map = {
    '0A': 'NOT REFERENCED from reset',
    '0B': 'NOT REFERENCED from HOMING',
    '0C': 'NOT REFERENCED from CONFIGURATION',
    '0D': 'NOT REFERENCED from DISABLE',
    '0E': 'NOT REFERENCED from READY',
    '0F': 'NOT REFERENCED from MOVING',
    '10': 'NOT REFERENCED ESP stage error',
    '11': 'NOT REFERENCED from JOGGING',
    '14': 'CONFIGURATION',
    '1E': 'HOMING commanded from RS-232-C',
    '1F': 'HOMING commanded by SMC-RC',
    '28': 'MOVING',
    '32': 'READY from HOMING',
    '33': 'READY from MOVING',
    '34': 'READY from DISABLE',
    '35': 'READY from JOGGING',
    '3C': 'DISABLE from READY',
    '3D': 'DISABLE from MOVING',
    '3E': 'DISABLE from JOGGING',
    '46': 'JOGGING from READY',
    '47': 'JOGGING from DISABLE',
    '48': 'SIMULATION MODE'}

STAGES_DEFAULT_PARAMS = {'MFA-CC': {'unit':            'mm',
                                    'travel_range':    25,
                                    'max_speed':       2.5},
                         'PR50CC': {'unit':            'deg',
                                    'travel_range':    360,
                                    'max_speed':       20}}


# never wait for more than this e.g. during wait_states
MAX_WAIT_TIME_SEC = 12

# time to wait after sending a command. This number has been arrived at by
# trial and error
# LU: this time still fails sometimes... TODO: fix
COMMAND_WAIT_TIME_SEC = 0.06

# States from page 65 of the manual
STATE_NOT_REFERENCED_FROM_RESET = '0A'
STATE_NOT_REFERENCED_FROM_CONFIGURATION = '0C'
STATE_NOT_REFERENCED_FROM_MOVING = '0F'
STATE_READY_FROM_HOMING = '32'
STATE_READY_FROM_MOVING = '33'

STATE_CONFIGURATION = '14'

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
    def __init__(self, state):
        super(SMC100DisabledStateException, self).__init__('Disabled state encountered: '+state)

class SMC100RS232CorruptionException(Exception):
    def __init__(self, c):
        super(SMC100RS232CorruptionException, self).__init__('RS232 corruption detected: {}'.format(hex(ord(c))))

class SMC100InvalidResponseException(Exception):
    def __init__(self, cmd, resp):
        s = 'Invalid response to {}: {}'.format(cmd, resp)
        super(SMC100InvalidResponseException, self).__init__(s)

class SMC100Connection:
    _sleepfunc = time.sleep
    def __init__(self, port, silent=True):
        self._port = None
        # super(SMC100Connection, self).__init__()
        self._silent = silent
        print('Connecting to SMC100 on {}'.format(port))
        self.COMport = port
        self._port = serial.Serial(
            port = port,
            baudrate = 57600,
            bytesize = 8,
            stopbits = 1,
            parity = 'N',
            xonxoff = True,
            timeout = 0.050)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._port:
            self._port.close()
            self._port = None

    def __del__(self):
        if self._port:
            self._port.close()
            self._port = None

class SMC100Stage(SMC100Connection):
    """
    Class for individual SMC100 stages, inheriting instance of SMC100 connection.

    Accepts commands in the form of:

    <ID><command><arguments><CR><LF>

    Reply, if any, will follow

    <ID><command><result><CR><LF>

    There is minimal support for manually setting stage parameter as Newport's
    ESP stages can supply the SMC100 with the correct configuration parameters.
    """
    def __init__(self, parent, smcID, stage_defaults=None):
        assert smcID is not None
        self._last_sendcmd_time = 0
        self.obj = parent
        self._smcID = str(smcID)
        print('> Added stage {} to the connection on port {}'.format(smcID, self.COMport))
        if stage_defaults is not None:
            print('> Setting defaults for stage type {}'.format(stage_defaults))
            for k, v in STAGES_DEFAULT_PARAMS[stage_defaults].items():
                setattr(self, k, v)

    def __getattr__(self,attr):
        return getattr(self.obj, attr)

    def set_defaults(self, name=None):
        if name is None:
            self.name = self.sendcmd(command='ZX',argument='?',expect_response=True)
        else:
            self.name = name
        try:
            for k, v in STAGES_DEFAULT_PARAMS[self.name].items():
                setattr(self, k, v)
        except Exception as ex:
            raise ex

    def sendcmd(self, command, argument=None, expect_response=False, retry=False):
        """
        Send the specified command along with the argument, if any. The response
        is checked to ensure it has the correct prefix, and is returned WITHOUT
        the prefix.

        It is important that for GET commands, e.g. 1ID?, the ? is specified as an
        ARGUMENT, not as part of the command. Doing so will result in assertion
        failure.

        If expect_response is True, a response is expected from the controller
        which will be verified and returned without the prefix.

        If expect_response is True, and retry is True or an integer, then when the
        response does not pass verification, the command will be sent again for
        retry number of times, or until success if retry is True.

        The retry option MUST BE USED CAREFULLY. It should ONLY be used read-only
        commands, because otherwise REPEATED MOTION MIGHT RESULT. In fact some
        commands are EXPLICITLY REJECTED to prevent this, such as relative move.
        """
        assert command[-1] != '?'

        if self._port is None:
            return

        if argument is None:
            argument = ''

        prefix = self._smcID + command
        tosend = prefix + str(argument)
        print(tosend)
        # prevent certain commands from being retried automatically
        no_retry_commands = ['PR', 'OR']
        if command in no_retry_commands:
            retry = False

        while self._port is not None:
            if expect_response:
                self._port.flushInput()

            self._port.flushOutput()
            self._port.write(tosend.encode())
            self._port.write(b'\r\n')
            self._port.flush()

            if not self._silent:
                self._emit('sent', tosend)

            if expect_response:
                try:
                    response = self._readline()
                    if response.startswith(prefix):
                        return response[len(prefix):]
                    else:
                        raise SMC100InvalidResponseException(command, response)
                except Exception as ex:
                    if not retry or retry <=0:
                        raise ex
                    else:
                        if type(retry) == int:
                            retry -= 1
                        continue
            else:
                # we only need to delay when we are not waiting for a response
                now = time.time()
                dt = now - self._last_sendcmd_time
                dt = COMMAND_WAIT_TIME_SEC - dt
                if dt > 0:
                    self._sleepfunc(dt)

                self._last_sendcmd_time = now
                return None

    def _readline(self):
        """
        Returns a line, that is reads until line termination \r\n. Required due to
        different behaviour of serial

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
            c = self._port.read().decode()
              # ignore \r since it is part of the line terminator
            if len(c) == 0:
                raise SMC100ReadTimeOutException()
            elif c == '\r':
                continue
            elif c == '\n':
                done = True
            elif ord(c) > 32 and ord(c) < 127:
                line += c
            else:
                raise SMC100RS232CorruptionException(c)

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
            print('[SMC100' + prefix + '] ' + message)

    def wait_states(self, targetstates, ignore_disabled_states=False):
        """
        Waits for the controller to enter one of the the specified target state.
        Controller state is determined via the TS command.

        If ignore_disabled_states is True, disable states are ignored. The normal
        behaviour when encountering a disabled state when not looking for one is
        for an exception to be raised.

        Note that this method will ignore read timeouts and keep trying until the
        controller responds.  Because of this it can be used to determine when the
        controller is ready again after a command like PW0 which can take up to 10
        seconds to execute.

        If any disable state is encountered, the method will raise an error,
        UNLESS you were waiting for that state. This is because if we wait for
        READY_FROM_MOVING, and the stage gets stuck we transition into
        DISABLE_FROM_MOVING and then STAY THERE FOREVER.
        """
        starttime = time.time()
        done = False
        self._emit('Waiting for states {}'.format(str(targetstates)))
        while not done:
            waittime = time.time() - starttime
            if waittime > MAX_WAIT_TIME_SEC:
                raise SMC100WaitTimedOutException()
            try:
                state = self.get_status()[1]
                if state in targetstates:
                    self._emit('in state {}'.format(state))
                    return state
                elif not ignore_disabled_states:
                    disabledstates = [
                      STATE_DISABLE_FROM_READY,
                      STATE_DISABLE_FROM_JOGGING,
                      STATE_DISABLE_FROM_MOVING]
                    if state in disabledstates:
                        raise SMC100DisabledStateException(state)
                elif state in ['0B','0C','0D','0E','0F']: # Not referenced
                    self._emit('not referenced')
                    return state

            except SMC100ReadTimeOutException:
                self._emit('Read timed out, retrying in 1 second')
                self._sleepfunc(1)
                continue
    def reset_and_configure(self):
        """
        Configures the controller by resetting it and then asking it to load
        stage parameters from an ESP compatible stage. This is then followed
        by a homing action.
        """
        self.sendcmd(command='RS')
        self._sleepfunc(3)

        self.wait_states(STATE_NOT_REFERENCED_FROM_RESET, ignore_disabled_states=True)

        try:
            stage = self.sendcmd(command='ID', argument='?', expect_response=True)
            print('Found stage', stage)
        except:
            self.move_absolute(0, waitStop=False)

        # enter config mode
        self.sendcmd(command='PW', argument=1)
        self.wait_states(STATE_CONFIGURATION)
        # load stage parameters
        self.sendcmd(command='ZX', argument=1)
        # enable stage ID check
        self.sendcmd(command='ZX', argument=2)
        # exit configuration mode
        self.sendcmd(command='PW', argument=0)

        # wait for us to get back into NOT REFERENCED state
        self.wait_states(STATE_NOT_REFERENCED_FROM_CONFIGURATION)
        self.home(waitStop=True)

    def get_status(self, silent=False):
        """
        Executes TS? and returns the the error code as integer and state as string
        as specified on pages 64 - 65 of the manual.
        """

        resp = self.sendcmd(command='TS', argument='?', expect_response=True, retry=10)
        errors = int(resp[0:4], 16)
        state = resp[4:]

        assert len(state) == 2

        if not silent:
            print('status:')
            msg = '  state: {}'.format(controller_state_map[state])
            print(msg)
        return errors, state

    def get_position(self):
        pos = float(self.sendcmd(command='TP', argument='?', expect_response=True, retry=10))
        return pos

    def home(self, waitStop=True):
        """
        Homes the controller. If waitStop is True, then this method returns when
        homing is complete.

        Note that because calling home when the stage is already homed has no
        effect, and homing is generally expected to place the stage at the
        origin, an absolute move to 0 um is executed after homing. This ensures
        that the stage is at origin after calling this method.

        Calling this method is necessary to take the controller out of not referenced
        state after a restart.
        """
        self.sendcmd(command='OR')
        if waitStop:
          # wait for the controller to be ready
            st = self.wait_states((STATE_READY_FROM_HOMING,
                                    STATE_READY_FROM_MOVING))
            if st == STATE_READY_FROM_MOVING:
                self.move_absolute(0, waitStop=True)
            else:
                self.move_absolute(0, waitStop=False)
                st = self.wait_states((STATE_READY_FROM_MOVING,
                                        STATE_NOT_REFERENCED_FROM_MOVING))
                if st == STATE_NOT_REFERENCED_FROM_MOVING:
                    self.home(waitStop=True)

    def move_relative(self, distance, waitStop=True):
        """
        Moves the stage relatively to the current position by the given distance.

        If waitStop is True then this method returns when the move is completed.
        """
        current_pos = get_position()

        # assert current_pos + distance < self.travel_range
        # assert current_pos + distance > 0

        # if dist < 0: # Avoid
        #     dist = max(dist, -current_pos) + buffer
        # else:
        #     dist = min(dist, self.travel_range-current_pos) - buffer

        self.sendcmd(command='PR', argument=distance)
        if waitStop:
            # If we were previously homed, then something like PR0 will have no
            # effect and we end up waiting forever for ready from moving because
            # we never left ready from homing. This is why STATE_READY_FROM_HOMING
            # is included.
            self.wait_states((STATE_READY_FROM_MOVING, STATE_READY_FROM_HOMING))

    def move_absolute(self, position, waitStop=True, retry=True):
        """
        Moves the stage to the given absolute position.

        If waitStop is True then this method returns when the move is completed.
        """

        # assert position < self.travel_range
        # assert not position < 0 # When homing it goes to -0.0, which, apparently, is not 0...

        self.sendcmd('PA', position)

        if waitStop:
            # If we were previously homed, then something like PR0 will have no
            # effect and we end up waiting forever for ready from moving because
            # we never left ready from homing. This is why STATE_READY_FROM_HOMING
            # is included.
            st = self.wait_states((STATE_READY_FROM_MOVING,
                                   STATE_READY_FROM_HOMING,
                                   STATE_NOT_REFERENCED_FROM_MOVING))
            if st == STATE_NOT_REFERENCED_FROM_MOVING and retry:
                self.home()
        def enable(self):
        st = self.get_status(silent=True)
        if st[1] not in ['3C','3D','3E']:
            print('Stage not in DISABLE state, trying to enable anyway')
        self.sendcmd(command='MM',argument=1)

    def disable(self):
        st = self.get_status(silent=True)
        if st[1] not in ['32','33', '34', '35']:
            print('Stage not in READY state, trying to disable anyway')
        self.sendcmd(command='MM',argument=0)
        
    def stop(self):
        self.sendcmd(command='ST')
