import sys
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from woodox_ros2_interface.srv import Damp, StandUp, GetDown, Ready, Move, SetMode, GetMode, GetState

class WdxClient(Node):
    def __init__(self):
        super().__init__('WDXClientPy')
        self.standUpClient = self.create_client(StandUp, 'StandUp')
        self.getDownClient = self.create_client(GetDown, 'GetDown')
        self.dampClient = self.create_client(Damp, 'Damp')
        self.moveClient = self.create_client(Move, 'Move')
        self.readyClient = self.create_client(Ready, 'Ready')
        self.getModeClient = self.create_client(GetMode, 'GetMode')
        self.setModeClient = self.create_client(SetMode, 'SetMode')
        self.getStateClient = self.create_client(GetState, 'GetState')
        self.lock = threading.Lock()

    def standUp(self, timeout=6.0):
        req = StandUp.Request()
        future = self.standUpClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("standUp success.")
                else:
                    self.get_logger().error(f"standUp error. code={code}")
            else:
                self.get_logger().error("standUp exception.")
        else:
            self.get_logger().error('standUp timeout.')
            future.cancel()

        return code

    def getDown(self, timeout=6.0):
        req = GetDown.Request()
        future = self.getDownClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("getDown success.")
                else:
                    self.get_logger().error(f"getDown error. code={code}")
            else:
                self.get_logger().error("getDown exception.")
        else:
            self.get_logger().error('getDown timeout.')
            future.cancel()

        return code

    def ready(self, timeout=6.0):
        req = Ready.Request()
        req.gait = 0
        future = self.readyClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("ready success.")
                else:
                    self.get_logger().error(f"ready error. code={code}")
            else:
                self.get_logger().error("ready exception.")
        else:
            self.get_logger().error('ready timeout.')
            future.cancel()

        return code

    def damp(self, timeout=6.0):
        req = Damp.Request()
        future = self.dampClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("damp success.")
                else:
                    self.get_logger().error(f"damp error. code={code}")
            else:
                self.get_logger().error("damp exception.")
        else:
            self.get_logger().error('damp timeout.')
            future.cancel()

        return code

    def move(self, vx, vy, vyaw, timeout=0.5):
        req = Move.Request()
        req.vx = vx
        req.vy = vy
        req.vyaw = vyaw

        code = 1

        future = self.moveClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("move success.")
                else:
                    self.get_logger().error(f"move error. code={code}")
            else:
                self.get_logger().error("move exception.")
        else:
            self.get_logger().error('move timeout.')
            future.cancel()

        return code

    def getMode(self, timeout=3.0):
        req = GetMode.Request()
        future = self.getModeClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1
        mode = 0

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    mode = rsp.mode
                    self.get_logger().info("getmode success. mode={mode}")
                else:
                    self.get_logger().error(f"getmode error. code={code}")
            else:
                self.get_logger().error("getmode exception.")
        else:
            self.get_logger().error('getmode timeout.')
            future.cancel()

        return code,mode

    def setMode(self, mode, timeout=3.0):
        req = SetMode.Request()
        req.mode = mode

        code = 1

        future = self.moveClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    self.get_logger().info("setmode success. mode={mode}")
                else:
                    self.get_logger().error(f"setmode error. code={code}")
            else:
                self.get_logger().error("setmode exception.")
        else:
            self.get_logger().error('setmode timeout.')
            future.cancel()

        return code

    def getState(self, timeout=3.0):
        req = GetState.Request()
        future = self.getStateClient.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        code = 1
        state = "Unknown"

        if future.done():
            rsp = future.result()
            if rsp is not None:
                code = rsp.code
                if code == 0:
                    state = rsp.state
                    self.get_logger().info("getState success. state={state}")
                else:
                    self.get_logger().error(f"getState error. code={code}")
            else:
                self.get_logger().error("getState exception.")
        else:
            self.get_logger().error('getState timeout.')
            future.cancel()

        return code,state

def main(args = None):
    rclpy.init(args = args)
    cli = WdxClient()

    try:
        rc = cli.standUp()
        if rc == 0:
            print(f"client standUp success.")
        else:
            print(f"client standUp error. rc={rc}")

        rc = cli.ready()
        if rc == 0:
            print(f"client ready success.")
        else:
            print(f"client ready error. rc={rc}")

        rc,mode = cli.getMode()
        if rc == 0:
            print(f"client getMode success. mode={rc}")
        else:
            print(f"client getMode error. rc={rc}")

        rc = cli.setMode(0)
        if rc == 0:
            print(f"client setMode success.")
        else:
            print(f"client setMode error. rc={rc}")

        rc,state = cli.getState()
        if rc == 0:
            print(f"client getState success. state={state}")
        else:
            print(f"client getState error. rc={rc}")

        rc = cli.move(0.5, 0.0, 0.0)
        if rc == 0:
            print(f"client move success.")
        else:
            print(f"client move error. rc={rc}")

        rc = cli.getDown()
        if rc == 0:
            print(f"client getDown success.")
        else:
            print(f"client getDown error. rc={rc}")

        rc = cli.damp()
        if rc == 0:
            print(f"client damp success.")
        else:
            print(f"client damp error. rc={rc}")

    finally:
        print("cli destroy ...")
        cli.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
