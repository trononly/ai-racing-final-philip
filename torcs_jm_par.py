import getopt
import json
import os
import socket
import sys
import time

PI = 3.14159265359

data_size = 2**17

ophelp = "Options:\n"
ophelp += " --host, -H <host>    TORCS server host. [localhost]\n"
ophelp += " --port, -p <port>    TORCS port. [3001]\n"
ophelp += " --id, -i <id>        ID for server. [SCR]\n"
ophelp += " --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n"
ophelp += " --episodes, -e <#>   Maximum learning episodes. [1]\n"
ophelp += (
    " --track, -t <track>  Your name for this track. Used for learning. [unknown]\n"
)
ophelp += " --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n"
ophelp += " --parameters, -f <#>   Input parameters of target_speed : int, steer_gain : int, centering_gain : float, brake_threshold : float, 6*[gear_speeds : int] (dont put brackets in the input tho), enable_traction_control : bool. [1]\n"
ophelp += " --debug, -d          Output full telemetry.\n"
ophelp += " --help, -h           Show this help.\n"
ophelp += " --version, -v        Show current version."
usage = f"Usage: {sys.argv[0]} [ophelp [optargs]] \n"
usage = usage + ophelp
version = "20130505-2"

#### Parameters

#############################################
# MODULAR DRIVE LOGIC WITH USER PARAMETERS  #
#############################################

import math

# ================= USER CONFIGURABLE PARAMETERS =================

######### REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE #########
TARGET_SPEED = 300  # Race defaults (no sliders in race mode): max unlock, corners capped by adaptive logic.
STEER_GAIN = 22  # Race defaults.
CENTERING_GAIN = 0.20  # Race defaults.
BRAKE_THRESHOLD = (
    0.40  # Race defaults: 23deg is a spin; 14deg (old 0.25) is normal S-turn cornering.
)
GEAR_SPEEDS = [
    0,
    50,
    80,
    110,
    150,
    185,
]  # Race defaults: 1st=0 locked, taller for momentum.
ENABLE_TRACTION_CONTROL = True  # Toggle traction control system.
######### END OF REQUIRED #########

# User configurable parameters as cli input:
# python torcs_jm_par.py --parameters [json data]


# ================= HELPER FUNCTIONS =================

# Sensor-only by design; the earlier track-memory planner was removed.


def _fallback_target(S):
    """Entry speed from centre fan, side fan only for real corners.
    Centre (+-1deg) reads 200 on straights. Side fan is only trusted when
    asymmetric (one side much shorter = real corner), not when both sides
    are equally short (straight road edges)."""
    if abs(S.get("trackPos", 0)) > 1.0:
        return min(TARGET_SPEED, 40.0)
    track = S.get("track", [])
    if len(track) < 19:
        return 300.0

    # Centre fan: idx 7..11 = -1, -0.5, 0, 0.5, 1 degrees
    center_fan = [t for t in track[7:12] if t > 0]
    if not center_fan:
        return 300.0
    _sorted_center = sorted(center_fan)
    d_center = _sorted_center[len(_sorted_center) // 2]

    # Side fan split left/right to detect asymmetry. Narrow (+-19deg) on
    # straights so off-centre +-45deg wall touches can't false-trigger;
    # widen to +-45deg when the centre reads <100 (corner complex).
    if d_center < 100:
        left_fan = [t for t in track[0:7] if t > 0]  # -45..-1.7deg
        right_fan = [t for t in track[12:19] if t > 0]  # +1.7..+45deg
    else:
        left_fan = [t for t in track[1:7] if t > 0]  # -19..-1.7deg
        right_fan = [t for t in track[12:18] if t > 0]  # +1.7..+19deg
    d_left = min(left_fan) if left_fan else d_center
    d_right = min(right_fan) if right_fan else d_center
    d_side = min(d_left, d_right)
    asym = abs(d_left - d_right)

    # Side reading only counts when much closer than centre and asymmetric.
    # Straight: centre 200, sides similar -> centre. Corner: inside short,
    # outside long -> side.
    if d_side < d_center * 0.5 and asym > 15.0 and d_center < 150 and abs(S.get("trackPos", 0)) < 0.5:
        d = d_side
    else:
        d = d_center

    if d > 160:
        v_corner = 300.0
    elif d > 60:
        v_corner = 140.0 + (d - 60.0) * 1.6  # 140 .. 300
    elif d > 25:
        v_corner = 80.0 + (d - 25.0) * 1.7  # 80 .. 140
    else:
        v_corner = max(55.0, 55.0 + d * 1.0)  # 55 .. 80

    v_c_ms = v_corner / 3.6
    d_eff = max(0.0, d - 8.0)  # 8 m reaction/transition margin
    v_now_max_ms = math.sqrt(v_c_ms * v_c_ms + 2.0 * 11.0 * d_eff)
    return min(float(TARGET_SPEED), v_now_max_ms * 3.6)


def _fallback_line(S):
    """Sensor-only line: outside on entry, sweep to apex, unwind on exit.
    Sign: diff>0 = left-hander = outside is negative trackPos."""
    track = S.get("track", [])
    if len(track) < 17:
        return 0.0
    lefts = [t for t in track[1:6] if t > 0]  # -19..-2.5deg
    rights = [t for t in track[13:18] if t > 0]  # +2.5..+19deg
    if not lefts or not rights:
        return 0.0
    l_mean = sum(lefts) / len(lefts)
    r_mean = sum(rights) / len(rights)
    diff = r_mean - l_mean
    center = track[9] if track[9] > 0 else 200.0
    if abs(diff) < 5 and center > 60:
        return 0.0
    sgn = -1.0 if diff > 0 else 1.0
    if center > 100:
        return sgn * 0.55
    elif center > 45:
        return sgn * 0.75
    else:
        return sgn * 0.90


def get_adaptive_target(S):
    # Centre fan for straights, asymmetric side fan for corners.
    speed = _fallback_target(S)
    if abs(S.get("trackPos", 0)) > 1.0:
        speed = min(speed, 40.0)
    return min(speed, float(TARGET_SPEED))


# Stuck recovery: TORCS stucktimer counts steps the car is not progressing forward.
STUCK_THRESHOLD = 60  # steps (~1.2 sec) before recovery is attempted
STUCK_REVERSE_STEPS = (
    80  # how many steps to drive in reverse (capped to 35 at call sites)
)


def calculate_steering(S):
    tp = S.get("trackPos", 0.0)
    # Off-track: forget racing line, just get back on.
    if abs(tp) > 1.0:
        gain = CENTERING_GAIN * 2.0 + 0.5
        steer = (S["angle"] * STEER_GAIN / math.pi) - (tp * gain)
        speed = S.get("speedX", 0)
        return max(-1, min(1, steer / (1.0 + max(0.0, speed) * 0.0055)))
    gain = CENTERING_GAIN
    if abs(tp) > 0.7:
        gain = CENTERING_GAIN * 1.5 + 0.15
    # Racing line: chase moving target, not center 0. Wide entry, clip apex.
    # EMA (~140ms) absorbs slow sensor swing; real entries develop slower.
    line_target = _fallback_line(S)
    global _prev_line_target
    line_target = 0.92 * _prev_line_target + 0.08 * line_target
    _prev_line_target = line_target
    err_tp = tp - line_target
    steer = (S["angle"] * STEER_GAIN / math.pi) - (err_tp * gain)
    # Speed-sensitive divisor keeps high-speed steering calm.
    speed = S.get("speedX", 0)
    steer = steer / (1.0 + max(0.0, speed) * 0.0055)
    steer = max(-1, min(1, steer))
    # Light smoothing kills mode-switch flicker. 0.65/0.35 keeps authority.
    global _prev_steer
    steer = 0.65 * steer + 0.35 * _prev_steer
    _prev_steer = steer
    return max(-1, min(1, steer))


def calculate_throttle(S, R):
    # Only cut throttle when actually moving — a stationary car needs
    # power to straighten out; cutting it locks the car in place.
    # Gate at 15 (launch releases at 10): the 10-15 band coasts with no
    # interference so momentum survives instead of oscillating at the gate.
    if abs(S.get("angle", 0)) > BRAKE_THRESHOLD and S.get("speedX", 0) > 15:
        return 0.0
    target = get_adaptive_target(S)
    speed = S.get("speedX", 0)
    steer_pen = abs(R.get("steer", 0)) * 6.0
    err = (target - steer_pen) - speed
    # Hotter floor pushes on exits instead of coasting through corners.
    if err > 0:
        accel = min(1.0, 0.55 + err * 0.05)  # err>9 -> full immediately
    else:
        accel = max(0.0, 0.60 + err * 0.03)  # err=-8 -> 0.36, not 0.10
        if err < -25:
            accel = 0.0
    if speed < 10:
        accel += 1 / (speed + 0.1)
    # Cap low-speed throttle: full power spins the wheels in 1st and TC
    # chatters. 0.65 pulls cleanly without spinning. Applies at speed 0
    # too (static breakaway is where wheels spin most freely).
    if speed < 30:
        accel = min(accel, 0.65)
    return max(0.0, min(1.0, accel))


def apply_brakes(S, R):
    """Sensor-only brake: yaw safety, then over-target fallback braking."""
    speed = S.get("speedX", 0)
    ang = abs(S.get("angle", 0))
    # Only yaw-brake when moving with real momentum; a slow car with
    # residual yaw is a stall, not a spin, and needs throttle not brake.
    # Gate at 15 (launch releases at 10) so momentum survives the crossing.
    # Soft tiers leave front-tire grip for steering.
    # Thresholds relative to BRAKE_THRESHOLD so --parameters sliders stay live;
    # defaults (0.40): >0.40 -> 0.2, >0.55 -> 0.4, >0.75 -> 0.7.
    if ang > BRAKE_THRESHOLD and speed > 15:
        if ang > BRAKE_THRESHOLD + 0.35:
            return 0.7
        if ang > BRAKE_THRESHOLD + 0.15:
            return 0.4
        return 0.2
    if speed < 20:
        return 0.0

    target = get_adaptive_target(S)
    over = speed - (target - 5)
    if over < 2:
        return 0.0
    brake = min(0.95, (over - 2) / 18.0)
    # Trail-brake taper: shed braking force as steering demand rises,
    # so full brake + full steer can't be requested at once.
    steer_demand = abs(R.get("steer", 0))
    brake *= max(0.5, 1.0 - steer_demand * 0.4)
    return brake


def shift_gears(S, current_gear=1):
    """RPM up, slider-speed down. Downshift reads GEAR_SPEEDS (sliders) with a
    15 km/h guard band so it can't flicker with the RPM up."""
    rpm = S.get("rpm", 0)
    speed = S.get("speedX", 0)
    try:
        g = int(current_gear)
    except (TypeError, ValueError):
        g = 1
    g = max(1, min(6, g))
    if speed < 15:
        return 1
    # Redline ~13k. 12000 lands near peak power without TC hanging on the limiter.
    # Anti-hunt: gate upshift on current-gear floor + no shift mid-spin.
    if rpm > 12000 and g < 6:
        try:
            cur_start = GEAR_SPEEDS[g - 1]
        except (IndexError, TypeError):
            cur_start = 0
        if speed > cur_start and abs(S.get("angle", 0)) < 0.4:
            return g + 1
    if g > 1:
        if rpm < 3800:
            return g - 1
        # GEAR_SPEEDS[g-1] is the speed this gear starts at. High gears get
        # a wider guard so they can engage and pull instead of hunting.
        try:
            thresh = GEAR_SPEEDS[g - 1]
        except (IndexError, TypeError):
            thresh = 0
        _guard = 30 if g >= 5 else 15
        if speed < thresh - _guard and rpm < 6500:
            return g - 1
    return g


def traction_control(S, accel):
    # Proportional cut: slip 3.5 threshold, range 6.0, never kill more than 60%.
    if ENABLE_TRACTION_CONTROL:
        # Launch fix: stay active below 10 km/h so wheelspin at launch is
        # corrected. Only bypass at genuine standstill in the first 5 steps
        # so the car can break away. Logged telemetry starts at step 10,
        # so CSV rows 0-4 already get full TC.
        if _telemetry_step <= 5 and S.get("speedX", 0) < 0.5:
            return max(0.0, accel)
        try:
            wsv = S.get("wheelSpinVel", [0, 0, 0, 0])
            slip = (wsv[2] + wsv[3]) - (wsv[0] + wsv[1]) if len(wsv) >= 4 else 0
        except (TypeError, IndexError):
            slip = 0
        if slip > 3.5:
            cut = min(accel, (slip - 3.5) / 6.0)
            accel = max(0.40 * accel, accel - cut)  # never kill more than 60%
    return max(0.0, accel)


def is_stuck(S):
    """Return True when TORCS reports the car has been stuck for too long."""
    return S.get("stucktimer", 0) > STUCK_THRESHOLD


def clip(v, lo, hi):
    if v < lo:
        return lo
    elif v > hi:
        return hi
    else:
        return v


def bargraph(x, mn, mx, w, c="X"):
    """Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with."""
    if not w:
        return ""  # No width!
    if x < mn:
        x = mn  # Clip to bounds.
    if x > mx:
        x = mx  # Clip to bounds.
    tx = mx - mn  # Total real units possible to show on graph.
    if tx <= 0:
        return "backwards"  # Stupid bounds.
    upw = tx / float(w)  # X Units per output char width.
    if upw <= 0:
        return "what?"  # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu = 0, 0, 0, 0
    if mn < 0:  # Then there is a negative part to graph.
        if x < 0:  # And the plot is on the negative side.
            negpu = -x + min(0, mx)
            negnonpu = -mn + x
        else:  # Plot is on pos. Neg side is empty.
            negnonpu = -mn + min(0, mx)  # But still show some empty neg.
    if mx > 0:  # There is a positive part to the graph
        if x > 0:  # And the plot is on the positive side.
            pospu = x - max(0, mn)
            posnonpu = mx - x
        else:  # Plot is on neg. Pos side is empty.
            posnonpu = mx - max(0, mn)  # But still show some empty pos.
    nnc = int(negnonpu / upw) * "-"
    npc = int(negpu / upw) * c
    ppc = int(pospu / upw) * c
    pnc = int(posnonpu / upw) * "_"
    return "[%s]" % (nnc + npc + ppc + pnc)


class Client:
    def __init__(
        self, H=None, p=None, i=None, e=None, t=None, s=None, d=None, vision=False
    ):
        self.vision = vision

        self.host = "localhost"
        self.port = 3001
        self.sid = "SCR"
        self.maxEpisodes = 1  # "Maximum number of learning episodes to perform"
        self.trackname = "unknown"
        self.stage = 3  # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug = False
        self.maxSteps = 100000  # 50steps/second
        self.parse_the_command_line()
        if H:
            self.host = H
        if p:
            self.port = p
        if i:
            self.sid = i
        if e:
            self.maxEpisodes = e
        if t:
            self.trackname = t
        if s:
            self.stage = s
        if d:
            self.debug = d
        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print("Error: Could not create socket...")
            sys.exit(-1)
        self.so.settimeout(1)

        n_fail = 5
        while True:
            a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"

            initmsg = "%s(init %s)" % (self.sid, a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata = str()
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode("utf-8")
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    # Windows + RaceRunner: TORCS is managed externally, don't pkill/sh.
                    if sys.platform.startswith("win"):
                        print(
                            "Waiting for RaceRunner TORCS on %d (no auto-relaunch on Windows)..."
                            % self.port
                        )
                        time.sleep(2.0)
                        n_fail = 5
                    else:
                        print("relaunch torcs")
                        os.system("pkill torcs")
                        time.sleep(1.0)
                        if self.vision is False:
                            os.system("torcs -nofuel -nodamage -nolaptime &")
                        else:
                            os.system("torcs -nofuel -nodamage -nolaptime -vision &")

                        time.sleep(1.0)
                        os.system("sh autostart.sh")
                        n_fail = 5
                n_fail -= 1

            identify = "***identified***"
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(
                sys.argv[1:],
                "H:p:i:m:e:t:s:f:dhv",
                [
                    "host=",
                    "port=",
                    "id=",
                    "steps=",
                    "episodes=",
                    "track=",
                    "stage=",
                    "parameters=",
                    "debug",
                    "help",
                    "version",
                ],
            )
        except getopt.error as why:
            print("getopt error: %s\n%s" % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == "-h" or opt[0] == "--help":
                    print(usage)
                    sys.exit(0)
                if opt[0] == "-d" or opt[0] == "--debug":
                    self.debug = True
                if opt[0] == "-H" or opt[0] == "--host":
                    self.host = opt[1]
                if opt[0] == "-i" or opt[0] == "--id":
                    self.sid = opt[1]
                if opt[0] == "-t" or opt[0] == "--track":
                    self.trackname = opt[1]
                if opt[0] == "-s" or opt[0] == "--stage":
                    self.stage = int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] == "-p" or opt[0] == "--port":
                    self.port = int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] == "-e" or opt[0] == "--episodes":
                    self.maxEpisodes = int(opt[1])
                if opt[0] == "-m" or opt[0] == "--steps":
                    self.maxSteps = int(opt[1])
                ######### REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE #########
                if opt[0] == "-f" or opt[0] == "--parameters":
                    global \
                        TARGET_SPEED, \
                        STEER_GAIN, \
                        CENTERING_GAIN, \
                        BRAKE_THRESHOLD, \
                        GEAR_SPEEDS, \
                        ENABLE_TRACTION_CONTROL
                    json_object = json.loads(opt[1])
                    TARGET_SPEED = int(json_object["target_speed"])
                    STEER_GAIN = int(json_object["steer_gain"])
                    CENTERING_GAIN = float(json_object["centering_gain"])
                    BRAKE_THRESHOLD = float(json_object["brake_threshold"])
                    GEAR_SPEEDS = [int(x) for x in json_object["gear_thresholds"]]
                    # RaceRunner sends 5 (2nd-6th), file uses 6 (1st=0 + 2nd-6th). Normalize.
                    if len(GEAR_SPEEDS) == 5:
                        GEAR_SPEEDS = [0] + GEAR_SPEEDS
                    ENABLE_TRACTION_CONTROL = json_object["traction_control"]
                    print(
                        f"Running with parameters:\ntarget speed={TARGET_SPEED}\nsteer gain={STEER_GAIN}\ncentering gain={CENTERING_GAIN}\nbrake threshold={BRAKE_THRESHOLD}\ngear speeds={GEAR_SPEEDS}\ntraction control enabled={ENABLE_TRACTION_CONTROL}"
                    )
                ########## END OF REQUIRED #########
                if opt[0] == "-v" or opt[0] == "--version":
                    print("%s %s" % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print(
                "Bad parameter '%s' for option %s: %s\n%s"
                % (opt[1], opt[0], why, usage)
            )
            sys.exit(-1)
        if len(args) > 0:
            print("Superflous input? %s\n%s" % (", ".join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        """Server's input is stored in a ServerState object"""
        if not self.so:
            return
        sockdata = str()

        while True:
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode("utf-8")
            except socket.error as emsg:
                print(".", end=" ")
            if "***identified***" in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif "***shutdown***" in sockdata:
                print(
                    (
                        (
                            "Server has stopped the race on %d. "
                            + "You were in %d place."
                        )
                        % (self.port, self.S.d["racePos"])
                    )
                )
                self.shutdown()
                return
            elif "***restart***" in sockdata:
                print("Server has restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata:  # Empty?
                continue  # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H")  # Clear for steady output.
                    print(self.S)
                break  # Can now return from this function.

    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so:
            return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1], str(emsg[0])))
            sys.exit(-1)
        if self.debug:
            print(self.R.fancyout())

    def shutdown(self):
        if not self.so:
            return
        print(
            (
                "Race terminated or %d steps elapsed. Shutting down %d."
                % (self.maxSteps, self.port)
            )
        )
        self.so.close()
        self.so = None


class ServerState:
    """What the server is reporting right now."""

    def __init__(self):
        self.servstr = str()
        self.d = dict()

    def parse_server_str(self, server_string):
        """Parse the server string."""
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip("(").rstrip(")").split(")(")
        for i in sslisted:
            w = i.split(" ")
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()

    def fancyout(self):
        """Specialty output for useful ServerState monitoring."""
        out = str()
        sensors = [  # Select the ones you want in the order you want them.
            "stucktimer",
            "fuel",
            "distRaced",
            "distFromStart",
            "opponents",
            "wheelSpinVel",
            "z",
            "speedZ",
            "speedY",
            "speedX",
            "targetSpeed",
            "rpm",
            "skid",
            "slip",
            "track",
            "trackPos",
            "angle",
        ]

        for k in sensors:
            if k not in self.d:
                out += "%s: --missing--\n" % k
                continue
            if type(self.d.get(k)) is list:  # Handle list type data.
                if k == "track":  # Nice display for track sensors.
                    strout = str()
                    raw_tsens = ["%.1f" % x for x in self.d["track"]]
                    strout += (
                        " ".join(raw_tsens[:9])
                        + "_"
                        + raw_tsens[9]
                        + "_"
                        + " ".join(raw_tsens[10:])
                    )
                elif k == "opponents":  # Nice display for opponent sensors.
                    strout = str()
                    for osensor in self.d["opponents"]:
                        if osensor > 190:
                            oc = "_"
                        elif osensor > 90:
                            oc = "."
                        elif osensor > 39:
                            oc = chr(int(osensor / 2) + 97 - 19)
                        elif osensor > 13:
                            oc = chr(int(osensor) + 65 - 13)
                        elif osensor > 3:
                            oc = chr(int(osensor) + 48 - 3)
                        else:
                            oc = "?"
                        strout += oc
                    strout = " -> " + strout[:18] + " " + strout[18:] + " <-"
                else:
                    strlist = [str(i) for i in self.d[k]]
                    strout = ", ".join(strlist)
            else:  # Not a list type of value.
                if k == "gear":  # This is redundant now since it's part of RPM.
                    gs = "_._._._._._._._._"
                    p = int(self.d["gear"]) * 2 + 2  # Position
                    l = "%d" % self.d["gear"]  # Label
                    if l == "-1":
                        l = "R"
                    if l == "0":
                        l = "N"
                    strout = gs[:p] + "(%s)" % l + gs[p + 3 :]
                elif k == "damage":
                    strout = "%6.0f %s" % (
                        self.d[k],
                        bargraph(self.d[k], 0, 10000, 50, "~"),
                    )
                elif k == "fuel":
                    strout = "%6.0f %s" % (
                        self.d[k],
                        bargraph(self.d[k], 0, 100, 50, "f"),
                    )
                elif k == "speedX":
                    cx = "X"
                    if self.d[k] < 0:
                        cx = "R"
                    strout = "%6.1f %s" % (
                        self.d[k],
                        bargraph(self.d[k], -30, 300, 50, cx),
                    )
                elif k == "speedY":  # This gets reversed for display to make sense.
                    strout = "%6.1f %s" % (
                        self.d[k],
                        bargraph(self.d[k] * -1, -25, 25, 50, "Y"),
                    )
                elif k == "speedZ":
                    strout = "%6.1f %s" % (
                        self.d[k],
                        bargraph(self.d[k], -13, 13, 50, "Z"),
                    )
                elif k == "z":
                    strout = "%6.3f %s" % (
                        self.d[k],
                        bargraph(self.d[k], 0.3, 0.5, 50, "z"),
                    )
                elif k == "trackPos":  # This gets reversed for display to make sense.
                    cx = "<"
                    if self.d[k] < 0:
                        cx = ">"
                    strout = "%6.3f %s" % (
                        self.d[k],
                        bargraph(self.d[k] * -1, -1, 1, 50, cx),
                    )
                elif k == "stucktimer":
                    if self.d[k]:
                        strout = "%3d %s" % (
                            self.d[k],
                            bargraph(self.d[k], 0, 300, 50, "'"),
                        )
                    else:
                        strout = "Not stuck!"
                elif k == "rpm":
                    g = self.d.get("gear", 0)
                    try:
                        if g < 0:
                            g = "R"
                        else:
                            g = "%1d" % g
                    except TypeError:
                        g = "?"
                    strout = bargraph(self.d[k], 0, 10000, 50, g)
                elif k == "angle":
                    asyms = [
                        "  !  ",
                        ".|'  ",
                        "./'  ",
                        "_.-  ",
                        ".--  ",
                        "..-  ",
                        "---  ",
                        ".__  ",
                        "-._  ",
                        "'-.  ",
                        r"'\.",
                        "'|.  ",
                        "  |  ",
                        "  .|'",
                        "  ./'",
                        "  .-'",
                        "  _.-",
                        "  __.",
                        "  ---",
                        "  --.",
                        "  -._",
                        "  -..",
                        r"  '\.",
                        "  '|.",
                    ]
                    rad = self.d[k]
                    deg = int(rad * 180 / PI)
                    symno = int(0.5 + (rad + PI) / (PI / 12))
                    symno = symno % (len(asyms) - 1)
                    strout = "%5.2f %3d (%s)" % (rad, deg, asyms[symno])
                elif k == "skid":  # A sensible interpretation of wheel spin.
                    wsv = self.d.get("wheelSpinVel", [0, 0, 0, 0])
                    frontwheelradpersec = wsv[0] if len(wsv) > 0 else 0
                    skid = 0
                    if frontwheelradpersec:
                        skid = (
                            0.5555555555 * self.d.get("speedX", 0) / frontwheelradpersec
                            - 0.66124
                        )
                    strout = bargraph(skid, -0.05, 0.4, 50, "*")
                elif k == "slip":  # A sensible interpretation of wheel spin.
                    wsv = self.d.get("wheelSpinVel", [0, 0, 0, 0])
                    frontwheelradpersec = wsv[0] if len(wsv) > 0 else 0
                    slip = 0
                    if frontwheelradpersec and len(wsv) >= 4:
                        slip = (wsv[2] + wsv[3]) - (wsv[0] + wsv[1])
                    strout = bargraph(slip, -5, 150, 50, "@")
                else:
                    strout = str(self.d[k])
            out += "%s: %s\n" % (k, strout)
        return out


class DriverAction:
    """What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)"""

    def __init__(self):
        self.actionstr = str()
        self.d = {
            "accel": 0.2,
            "brake": 0,
            "clutch": 0,
            "gear": 1,
            "steer": 0,
            "focus": [-90, -45, 0, 45, 90],
            "meta": 0,
        }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d["steer"] = clip(self.d["steer"], -1, 1)
        self.d["brake"] = clip(self.d["brake"], 0, 1)
        self.d["accel"] = clip(self.d["accel"], 0, 1)
        self.d["clutch"] = clip(self.d["clutch"], 0, 1)
        if self.d["gear"] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d["gear"] = 0
        if self.d["meta"] not in [0, 1]:
            self.d["meta"] = 0
        if (
            type(self.d["focus"]) is not list
            or min(self.d["focus"]) < -180
            or max(self.d["focus"]) > 180
        ):
            self.d["focus"] = 0

    def __repr__(self):
        self.clip_to_limits()
        out = str()
        for k in self.d:
            out += "(" + k + " "
            v = self.d[k]
            if not type(v) is list:
                out += "%.3f" % v
            else:
                out += " ".join([str(x) for x in v])
            out += ")"
        return out

    def fancyout(self):
        """Specialty output for useful monitoring of bot's effectors."""
        out = str()
        od = self.d.copy()
        od.pop("gear", "")  # Not interesting.
        od.pop("meta", "")  # Not interesting.
        od.pop("focus", "")  # Not interesting. Yet.
        for k in sorted(od):
            if k == "clutch" or k == "brake" or k == "accel":
                strout = ""
                strout = "%6.3f %s" % (od[k], bargraph(od[k], 0, 1, 50, k[0].upper()))
            elif k == "steer":  # Reverse the graph to make sense.
                strout = "%6.3f %s" % (od[k], bargraph(od[k] * -1, -1, 1, 50, "S"))
            else:
                strout = str(od[k])
            out += "%s: %s\n" % (k, strout)
        return out


def destringify(s):
    """makes a string into a value or a list of strings into a list of
    values (if possible)"""
    if not s:
        return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]


# ================= MAIN DRIVE FUNCTION =================
# Module state. Clocks are step counts; _telemetry_step advances once per
# drive_modular call because _telemetry() runs on every path through it.
_reverse_steps_remaining = 0
_telemetry_fp = None
_telemetry_step = 0
_no_reverse_until = 0
_prev_dist_raced = 0
_rows_this_file = 0
_last_sig = None  # frozen-frame detector for telemetry
_prev_steer = 0.0  # steering smoother state
_prev_line_target = 0.0  # EMA state for noisy line target


def _telemetry(S, R):
    """Append one CSV line every 10 steps. Resets on every new race
    (distRaced drops) so the file stays small."""
    global _telemetry_fp, _telemetry_step, _prev_dist_raced, _rows_this_file, _last_sig
    _telemetry_step += 1
    try:
        _dr = S.get("distRaced", 0)
        if (
            _telemetry_fp is None
            or _dr < _prev_dist_raced - 100
            or _rows_this_file > 2000
        ):
            if _telemetry_fp is not None:
                try:
                    _telemetry_fp.close()
                except Exception:
                    pass
            _path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "telemetry.csv"
            )
            _telemetry_fp = open(_path, "w")
            print(f"[telemetry] writing to {_path}")
            _telemetry_fp.write(
                "step,speedX,target,centerDist,minTrack,brake,accel,steer,gear,rpm,trackPos,angle,distRaced,lapDist,fOpp,sev,planDist\n"
            )
            _rows_this_file = 0
        _prev_dist_raced = _dr
    except Exception:
        pass
    if _telemetry_step % 10 != 0:
        return
    try:
        # Skip frozen post-finish frames (server stopped, same S re-logged).
        _sig = (S.get("speedX", 0), S.get("distRaced", 0), S.get("rpm", 0))
        if _sig == _last_sig:
            return
        _last_sig = _sig
        track = S.get("track", [])
        center = track[9] if len(track) > 9 else -1
        _tmin = -1.0
        try:
            _tv = [t for t in track if t is not None and t > 0]
            if _tv:
                _tmin = min(_tv)
        except Exception:
            pass
        _fo = -1.0
        try:
            _o = S.get("opponents", [])
            if len(_o) >= 24:
                _ff = [o for o in _o[12:24] if o is not None and o >= 0]
                if _ff:
                    _fo = min(_ff)
        except Exception:
            pass
        _plan = None  # planner removed
        _sev = ""
        _pdist = ""
        _telemetry_fp.write(
            "%d,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.3f,%s,%.0f,%.3f,%.3f,%.1f,%.1f,%.1f,%s,%s\n"
            % (
                _telemetry_step,
                S.get("speedX", 0),
                get_adaptive_target(S),
                center,
                _tmin,
                R.get("brake", 0),
                R.get("accel", 0),
                R.get("steer", 0),
                str(R.get("gear", "?")),
                S.get("rpm", 0),
                S.get("trackPos", 0),
                S.get("angle", 0),
                S.get("distRaced", 0),
                S.get("distFromStart", 0),
                _fo,
                _sev,
                _pdist,
            )
        )
        _telemetry_fp.flush()
        _rows_this_file += 1
    except Exception:
        pass


def drive_modular(c):
    global _reverse_steps_remaining, _telemetry_step, _no_reverse_until
    S, R = c.S.d, c.R.d

    # --- Moving backwards: stop first, never keep reversing to finish ---
    if S.get("speedX", 0) < -2 and _reverse_steps_remaining == 0:
        R["gear"] = 1
        R["accel"] = 0.0
        R["brake"] = 0.8
        R["steer"] = 0.0
        _telemetry(S, R)
        return

    # --- Stuck recovery (short: 35 steps, gentle, then MUST go forward) ---
    if _reverse_steps_remaining > 0:
        R["gear"] = -1
        # Cap reverse speed: 0.6 built -38 with 19k rpm scream.
        if S.get("speedX", 0) < -15:
            R["accel"] = 0.0
            R["brake"] = 0.4
        else:
            R["accel"] = 0.4
            R["brake"] = 0.0
        R["steer"] = 0.0  # reverse straight; steering while reversing causes spins
        _reverse_steps_remaining -= 1
        if _reverse_steps_remaining == 0:
            # Cooldown: no new reverse for next 150 steps (~3s). Force forward.
            _no_reverse_until = _telemetry_step + 150
        _telemetry(S, R)
        return

    # Cooldown active: drive forward even if stucktimer says stuck.
    _in_cooldown = _telemetry_step < _no_reverse_until
    if not _in_cooldown and is_stuck(S):
        print(
            f"[recovery] stuck detected (stucktimer={S.get('stucktimer', 0)}), reversing..."
        )
        _reverse_steps_remaining = min(STUCK_REVERSE_STEPS, 35)
        return
    elif not _in_cooldown and abs(S.get("trackPos", 0)) > 1.2 and S.get("speedX", 0) < 5:
        print(
            f"[recovery] off-track and stopped (trackPos={S.get('trackPos', 0):.2f}), reversing..."
        )
        _reverse_steps_remaining = min(STUCK_REVERSE_STEPS, 35)
        return

    # --- Normal drive (planner sets target + line, physics sets brake) ---
    R["steer"] = calculate_steering(S)
    R["brake"] = apply_brakes(S, R)
    # Reduce throttle proportionally when braking to avoid fighting the brakes
    R["accel"] = calculate_throttle(S, R) * (1.0 - R["brake"])
    R["accel"] = traction_control(S, R["accel"])
    # Traffic: hold momentum behind (8-20m: push, don't lift — TORCS has no
    # slipstream, sitting back just bleeds time). Close (<8m): side-step
    # around if a side is clear, else lift. Solo = no-op throughout.
    try:
        _opp = S.get("opponents", [])
        if len(_opp) >= 36:
            _front = [o for o in _opp[12:24] if o is not None and o >= 0]
            if _front:
                _fd = min(_front)
                # 8m at 150 is 0.19s to react — not enough. Side-step from
                # 15m; the old elif-push (x1.1, capped at 1.0) did nothing.
                if _fd < 15:
                    _left_clear = all(
                        o > 20 for o in _opp[6:12] if o is not None and o >= 0
                    )
                    _right_clear = all(
                        o > 20 for o in _opp[24:30] if o is not None and o >= 0
                    )
                    if _right_clear:
                        R["steer"] = max(-1, min(1, R["steer"] - 0.15))
                    elif _left_clear:
                        R["steer"] = max(-1, min(1, R["steer"] + 0.15))
                    else:
                        R["accel"] *= 0.5
                        if _fd < 5:
                            R["brake"] = max(R["brake"], 0.3)
    except Exception:
        pass
    R["gear"] = shift_gears(S, R.get("gear", 1))
    # Launch override: if nearly stopped, ensure strong (not full) power.
    # No yaw gate — a yawed stall needs power to straighten out. Slightly
    # off-track (up to 1.5) allowed; speed<10 bounds the risk. 0.7 floor
    # matches the low-speed cap (0.65) so launch doesn't re-trigger TC chatter.
    if 0 <= S.get("speedX", 0) < 10 and abs(S.get("trackPos", 0)) < 1.5:
        R["brake"] = 0.0
        R["accel"] = max(R["accel"], 0.7)
        R["gear"] = 1
    _telemetry(S, R)
    return


# ================= MAIN LOOP =================
if __name__ == "__main__":
    #### DO NOT HARCODE THE PORT
    C = Client()
    for step in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        drive_modular(C)
        C.respond_to_server()
    C.shutdown()
