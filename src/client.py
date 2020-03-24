import zmq
import zmq.utils.jsonapi as zmqJson

#zmq context need to be global
context = zmq.Context()

class LeapMotionPoller():

    sub_channel="0 "
    def __init__(self):
        self.zmq = context
        self.socket = self.zmq.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5556")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "0")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    def Poll(self):
        polledSockets=dict(self.poller.poll(timeout=500))
        #print(polledSockets)
        try:
            if self.socket in polledSockets and polledSockets[self.socket]==zmq.POLLIN:
                jsTxt = self.socket.recv_string(zmq.DONTWAIT)
                #print(jsTxt)
                jsObj=zmqJson.loads(jsTxt.lstrip(LeapMotionPoller.sub_channel))
                #print(jsObj["handType"])
                return jsObj
        except:
            return None
            pass
        return None

import sys, os
sys.path.append(os.path.join('..'))
from uarm.wrapper import SwiftAPI
from uarm.swift.protocol import SERVO_BOTTOM, SERVO_LEFT, SERVO_RIGHT, SERVO_HAND
class UarmActuator():

    uArm_MIN_XYZ=[150,-155,22]
    uArm_MAX_XYZ=[220,155,150]
    lp_MIN_XYZ=[-150,80,-50]
    lp_MAX_XYZ=[150,250,150]

    def __init__(self):
        try:
            self.uArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
            self.uArm.waiting_ready()
            self.uArm.reset()

            self.jsonTrack=[]
        except:
            print("[Error] UarmActuator failed to init, check Power and connection")
            raise

    def connect(self):
        try:
            self.uArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
            self.uArm.waiting_ready()
            self.uArm.reset()
            self.uArm.register_callback(period=2)
        except:
            pass

    def disconnect(self):

        try:
            self.deregister_callback()
            self.uArm.set_position(x=150,y=0,z=22, speed=1000000)
            #kludge to fix the async cmd, may consider to use the set_position callback
            import time
            time.sleep(2)
            self.uArm.set_servo_detach()
            self.uArm.disconnect()
        except:
            pass
        finally:
            pass

    def register_callback(self,period):
        self.uArm.register_report_position_callback(callback=self.pos_callback)
        self.uArm.set_report_position(interval=period)

    def deregister_callback(self):
        self.uArm.set_report_position(0)

    def pos_callback(self,ret):
        print('report pos: {}, time: {}'.format(ret[:3], time.time()))
        uarm_unclamped_coor = ret[:3]
        uarm_clampped_coor = UarmActuator.XYZclamping(uarm_unclamped_coor,UarmActuator.uArm_MIN_XYZ,UarmActuator.uArm_MAX_XYZ)
        print('uarm_unclamped_coor: {}, uarm_clampped_coor: {}'.format(uarm_unclamped_coor,uarm_clampped_coor))

        self.uarm_cur_pos=UarmActuator.toRelativePos(uarm_clampped_coor,UarmActuator.uArm_MIN_XYZ,UarmActuator.uArm_MAX_XYZ)
        print('uarm_cur_pos: {}'.format(uarm_cur_pos))

        #TODO check json

    def onLeapMotionUpdate(self,aJson):
        if aJson is not None:
            #print("[onLeapMotionUpdate] json: %s" % aJson)
            self.jsonTrack.append(aJson)
            if len(self.jsonTrack) > 100:
                self.jsonTrack.remove(self.jsonTrack[0])
        return

    @staticmethod
    def clamping(num,aMin,aMax):
        return min(aMax,max(num,aMin))

    @staticmethod
    def XYZclamping(V,mins,maxes):
        return [UarmActuator.clamping(V[i],mins[i],UarmActuator.maxes[i]) for i in [0,1,2]]

    @staticmethod
    def toRelativePos(cur,mins,maxes):
        toPercent = lambda cur, min, max: (cur-min)/(max-min)
        return list(map(toPercent,cur,mins,maxes))

    def mapper(self):
        #clipping the input
        #using relative unit span for comparison
        #if distance is large than 0.05, the drive the motor to the target point
        #leapmotion: (position_x,position_y,position_z) ~ uarm: (^y,z,^x)
        #leapmotion: clip([-150,150],[80,250],[-50,150])
        uarm_clipped_xyz = None
        try:
            pass
        except:
            print("[Exception] some error")
            pass


        
        return

shouldRun=True

def PollSocket(poller,actuator):
    while shouldRun:
        resultJson=poller.Poll()
        if resultJson is not None:
            actuator.onLeapMotionUpdate(resultJson)

import threading

def main():

    poller = LeapMotionPoller()
    actuator = None
    try:
        actuator = UarmActuator()
        actuator.connect()
    except:
        print("[Error] actuator some error, stopped")
        raise
    runner = threading.Thread(target = PollSocket, args=(poller,actuator))
    runner.start()

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        global shouldRun
        sys.stdin.readline()
        shouldRun = False
    except KeyboardInterrupt:
        pass
    finally:
        runner.join()
        print("Client disconnecting...")
        actuator.disconnect()
        poller.socket.disconnect("tcp://localhost:5556")
        context.destroy()

    return

if __name__ == "__main__":
    main()
