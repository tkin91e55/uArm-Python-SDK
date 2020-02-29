import sys, os
sys.path.append(os.path.join('..'))
from uarm.wrapper import SwiftAPI
from uarm.swift.protocol import SERVO_BOTTOM, SERVO_LEFT, SERVO_RIGHT, SERVO_HAND
import zmq
import zmq.utils.jsonapi as zmqJson

#zmq context need to be global
context = zmq.Context()

class LeapMotionPoller():

    sub_channel="0 "
    def __init__(self):
        self.zmq = context
        self.socket = self.zmq.socket(zmq.SUB)
        self.socket.connect("tcp://192.168.8.103:5556")
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
                print(jsObj["handType"])
        except:
            pass
    
class UarmActuator():
    pass



shouldRun=True

def PollSocket(poller):
    while shouldRun:
        poller.Poll()

import threading

def main():

    swiftArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
    poller = LeapMotionPoller()
    runner = threading.Thread(target = PollSocket, args=(poller,))
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
        poller.socket.disconnect("tcp://192.168.8.103:5556")
        context.destroy()

        swiftArm.set_position(x=150,y=0,z=22, speed=1000000)
        swiftArm.set_servo_detach()
        swiftArm.disconnect()
    return

if __name__ == "__main__":
    main()
