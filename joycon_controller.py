from pyjoycon import JoyCon, get_L_id
import time
import asyncio
from websockets.sync.client import connect
from os import linesep



host = "ws://192.168.4.1/ws"

time.sleep(2)
global websocket
# Websocket Connect
websocket = connect(host)
print("Connection Started")
time.sleep(2)

MAX_ANGLE = 40
MIN_ANGLE = -40

kp = 0.2
kd = 0.0
ki = 0.0

deadband = 5
global previous_time

joycon_id = get_L_id()
joycon = JoyCon(*joycon_id)
status = joycon.get_status()

sticks = status.get('analog-sticks')
leftStick = sticks.get('left')
horizontal_stick = leftStick.get('horizontal')
vertical_stick = leftStick.get('vertical')

buttons = status.get('buttons')
left_buttons = buttons.get('left')
d = left_buttons.get('down')
u = left_buttons.get('up')
l = left_buttons.get('left')
r = left_buttons.get('right')
sl = left_buttons.get('sl')
sr = left_buttons.get('sr')
ll = left_buttons.get('l')
z = left_buttons.get('zl')

global d_p,u_p,l_p,r_p,sl_p,sr_p,ll_p,z_p
d_p = d
u_p = u
l_p = l
r_p = r
sl_p = sl
sr_p = sr 
ll_p = ll
z_p = z

global pitch_offset
pitch_offset = 0


pitch_setpoint = 0
heading_setpoint = 0
pitch_target = 0
heading_target = 0




def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def update_stick_pos():
    status = joycon.get_status()
    sticks = status.get('analog-sticks')
    leftStick = sticks.get('left')
    global horizontal_stick
    global vertical_stick
    horizontal_stick = leftStick.get('horizontal')
    vertical_stick = leftStick.get('vertical')
    buttons = status.get('buttons')
    left_buttons = buttons.get('left')
    global d,u,l,r,sl,sr,ll,z
    d = left_buttons.get('down')
    u = left_buttons.get('up')
    l = left_buttons.get('left')
    r = left_buttons.get('right')
    sl = left_buttons.get('sl')
    sr = left_buttons.get('sr')
    ll = left_buttons.get('l')
    z = left_buttons.get('zl')
    
    
    
def Controller(dt, psp, ptg, pe, te):
    error = ptg - psp
    delta_error = error - previous_error
    te = te + error
    
    kp_term = kp * error
    kd_term = kd * delta_error / dt
    ki_term = ki * te * dt
    
    pe = error
    
    if (abs(error) <= 4):
        kp_term = 0.5 * error
    #    if (abs(error) <= 10):
    #        kp_term = 0.3 * error
    #        if(abs(error)<=2):
    #            kp_term = 0.5 * error
        
        
    
    pitch_setpoint = psp + (kp_term + kd_term + ki_term)
    return round(pitch_setpoint), pe, te
    
    

    
    

previous_error = 0
total_error = 0
previous_time = time.time()

while (True):
    current_time = time.time() 
    delta_time = current_time - previous_time
    update_stick_pos()
    
    if (d != d_p):
        print("D: ",d)
        if (d == 1):
            pitch_offset = pitch_offset + 0.25
            if (pitch_offset > 15):
                pitch_offset = 15
            # Websocket Message
            #websocket.send("4s" + str(pitch_offset) )
            print("Pitch Offset: ",pitch_offset)
        d_p = d
    if (u != u_p):
        print("U: ",u)
        if (u == 1):
            pitch_offset = pitch_offset - 0.25
            if (pitch_offset < -15):
                pitch_offset = -15
            # Websocket Message
            #websocket.send("4s" + str(pitch_offset) )
            print("Pitch Offset: ",pitch_offset)
        u_p = u
    if (l != l_p):
        print("L: ",l)
        if (l == 1):
            MAX_ANGLE = MAX_ANGLE - 1
            MIN_ANGLE = MIN_ANGLE + 1
            if (MAX_ANGLE < 10):
                MAX_ANGLE = 10
            if (MIN_ANGLE > -10):
                MIN_ANGLE = -10
            print("Max Angle: ",MAX_ANGLE)
            print("Min Angle: ",MIN_ANGLE)
        l_p = l
    if (r != r_p):
        print("R: ",r)
        if (r == 1):
            MAX_ANGLE = MAX_ANGLE + 1
            MIN_ANGLE = MIN_ANGLE - 1
            if (MAX_ANGLE > 50):
                MAX_ANGLE = 50
            if (MIN_ANGLE < -50):
                MIN_ANGLE = -50
            print("Max Angle: ",MAX_ANGLE)
            print("Min Angle: ",MIN_ANGLE)
        r_p = r
    if (sl != sl_p):
        print("SL: ",sl)
        if (sl == 1):
            deadband = deadband - 1
            if (deadband < 0):
                deadband = 0
            print("Deadband: ",deadband)
        sl_p = sl
    if (sr != sr_p):
        print("SR: ",sr)
        if (sr == 1):
            deadband = deadband + 1
            if (deadband > 15):
                deadband = 15
            print("Deadband: ",deadband)
        sr_p = sr
        
    if (ll != ll_p):
        print("LL: ",ll)
        if (ll == 1):
            # Websocket Message
            websocket.send("ENMOT")
            print("Motors ENABLED")
            time.sleep(0.5)
            websocket.send("ENCON")
            print("Controller ENABLED")
            
        ll_p = ll
    if (z != z_p):
        print("Z: ",z)
        if (z == 1):
            # Websocket Message
            websocket.send("DISCON")
            print("Controller DISABLED")
            time.sleep(0.5)
            websocket.send("DISMOT")
            print("Motors DISABLED")
        z_p = z

    
    heading_setpoint = -map_range(horizontal_stick,750,3125,MIN_ANGLE,MAX_ANGLE)
    pitch_target = map_range(vertical_stick,3380,1250,MIN_ANGLE,MAX_ANGLE)
    
    if (pitch_target <= deadband and pitch_target > 0):
        pitch_target = 0
    if (heading_setpoint <= deadband and heading_setpoint > 0):
        heading_setpoint = 0
    if (pitch_target >= -deadband and pitch_target < 0):
        pitch_target = 0
    if (heading_setpoint >= -deadband and heading_setpoint < 0):
        heading_setpoint = 0
    

    if (delta_time >= 0.1):
        pitch_setpoint = pitch_target*100
        #pitch_setpoint, previous_error, total_error = Controller(delta_time, pitch_setpoint, pitch_target, previous_error, total_error)
        #pitch_setpoint = pitch_setpoint*100
        #pitch_setpoint = Controller(delta_time, pitch_setpoint, pitch_target)
        print("Pitch Setpoing: ", pitch_setpoint,"      Heading Setpoing: ", heading_setpoint)
        # Websocket Message
        websocket.send("SP" + str(pitch_setpoint) + "," + str(heading_setpoint) + ",")
        previous_time = current_time
    
    
    
   