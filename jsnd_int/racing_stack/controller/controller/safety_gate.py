"""Latched software E-stop and restart approval for simple_mux (safety_mode only).

ROS-free so it can be unit tested; simple_mux feeds it joystick buttons, wheel
odometry and wall time, and asks it which host may drive.

This is a SOFTWARE stop. It depends on the Jetson, simple_mux, ackermann_to_vesc
and the VESC USB link all being alive. It does not replace the physical battery
/ power cut, which must stay within reach of a person during every run.

Hosts:
  'estop'     latched stop. simple_mux publishes speed 0 / steering 0 every tick.
  'none'      not engaged. simple_mux publishes speed 0 / steering 0 every tick.
  'autodrive' the controller command may pass (fresh, then the TTC estop).
There is no manual (humandrive) host: in safety_mode LB is an E-stop button.

Latch triggers: any estop button (default A=0, LB=4), the software estop topic,
and loss of /joy while engaged. Only request_reset() clears a latch, and only
when every button is released on a fresh /joy, the wheels have been still for
standstill_hold_s on fresh odom, and the latch is at least min_latch_s old.
Clearing returns to 'none'; engaging again needs a NEW press of the engage
button (default RB=5). With require_hold, releasing RB returns to 'none'.
"""


class SafetyGate:
    def __init__(self, estop_buttons=(0, 4), engage_button=5, joy_timeout_s=0.5,
                 min_latch_s=2.0, standstill_mps=0.05, standstill_hold_s=1.0,
                 odom_timeout_s=0.5, require_hold=True, start_latched=True):
        self.estop_buttons = tuple(int(b) for b in estop_buttons)
        self.engage_button = int(engage_button)
        if self.engage_button in self.estop_buttons:
            raise ValueError('engage button cannot also be an estop button')
        self.joy_timeout_s = float(joy_timeout_s)
        self.min_latch_s = float(min_latch_s)
        self.standstill_mps = float(standstill_mps)
        self.standstill_hold_s = float(standstill_hold_s)
        self.odom_timeout_s = float(odom_timeout_s)
        self.require_hold = bool(require_hold)
        self.host = 'none'
        self.latched = False
        self.reason = ''
        self.latched_at = None
        self.joy_time = None
        self.joy_buttons = ()
        self.engage_was_pressed = True   # no engage from a button already held at start
        self.odom_time = None
        self.still_since = None
        self.events = []                 # (time, text) for the node to log
        if start_latched:
            self.trigger('startup: restart approval required', 0.0)

    @staticmethod
    def _pressed(buttons, index):
        return len(buttons) > index and bool(buttons[index])

    def _event(self, now, text):
        self.events.append((now, text))

    def trigger(self, reason, now):
        if not self.latched:
            self._event(now, f'ESTOP LATCHED: {reason}')
            self.reason = reason
        self.latched = True
        self.latched_at = now
        self.host = 'estop'

    def on_joy(self, buttons, now):
        buttons = tuple(buttons)
        self.joy_time, self.joy_buttons = now, buttons
        pressed = [b for b in self.estop_buttons if self._pressed(buttons, b)]
        engage = self._pressed(buttons, self.engage_button)
        if pressed:
            self.trigger(f'button {pressed}', now)
        elif not self.latched:
            if engage and not self.engage_was_pressed:
                if self.host != 'autodrive':
                    self._event(now, 'autodrive engaged (engage button pressed)')
                self.host = 'autodrive'
            elif self.require_hold and not engage and self.host == 'autodrive':
                self._event(now, 'engage button released -> none (speed 0)')
                self.host = 'none'
        self.engage_was_pressed = engage

    def on_odom(self, speed, now):
        self.odom_time = now
        if abs(speed) < self.standstill_mps:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def update(self, now):
        joy_age = None if self.joy_time is None else now - self.joy_time
        if self.host == 'autodrive' and (joy_age is None or joy_age > self.joy_timeout_s):
            self.trigger(f'joy lost ({"never" if joy_age is None else f"{joy_age:.2f} s"})', now)
        return self.host

    def request_reset(self, now):
        """Operator restart approval. Returns (accepted, message)."""
        if not self.latched:
            return False, 'not latched'
        problems = []
        if now - self.latched_at < self.min_latch_s:
            problems.append(f'latched only {now - self.latched_at:.1f} s (< {self.min_latch_s} s)')
        if self.joy_time is None or now - self.joy_time > self.joy_timeout_s:
            problems.append('no fresh /joy (pad connected?)')
        elif any(self.joy_buttons):
            problems.append('release every pad button')
        if self.odom_time is None or now - self.odom_time > self.odom_timeout_s:
            problems.append('no fresh wheel odometry')
        elif self.still_since is None or now - self.still_since < self.standstill_hold_s:
            problems.append(f'wheels not still for {self.standstill_hold_s} s')
        if problems:
            return False, 'reset refused: ' + '; '.join(problems)
        self._event(now, f'ESTOP RESET approved (was: {self.reason}) -> none; press engage to start')
        self.latched, self.reason, self.host = False, '', 'none'
        self.engage_was_pressed = True   # needs a new press after release
        return True, 'estop cleared -> none. Release and press the engage button to start from 0.'
