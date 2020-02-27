import sys
import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, "0")

shouldRun=True
def PollSocket():
    while shouldRun:
        print(shouldRun)
        jsObj = socket.recv_string()
        print(jsObj)

import threading

def main():
    runner = threading.Thread(target = PollSocket)
    runner.start()

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        global shouldRun
        sys.stdin.readline()
        shouldRun = False
        print("shouldRun False...")
    except KeyboardInterrupt:
        pass
    finally:
        runner.join()
        print("Client disconnecting...")
        socket.disconnect("tcp://localhost:5556")
        context.destroy()
    return

if __name__ == "__main__":
    main()