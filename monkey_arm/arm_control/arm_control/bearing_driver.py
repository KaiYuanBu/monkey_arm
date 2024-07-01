"""
Python driver class for Arm's bearing.

Written by:
    1. Kean Hao
Last Updated: 8 September 2023
"""

import time
import threading
import can
from .can_util import *

class BearingDriver(can.listener.Listener):
    """Driver class for Arm's bearing controller."""

    def __init__(self, bus:can.ThreadSafeBus, node_id:int) -> None:
        """Initialize CAN communication with Arm bearing controller.

        Args:
            node_id (int): CANOpen node ID of target controller.
            timeout (float, optional): CAN message timeout in second.
            position_threshold (float, optional): Threshold for determining if
                target position is reached.
        """
        self.bus = bus
        self.node_id = node_id
        
        # Initialize variables
        self.current_position = 0.0
        self.is_running = False

        # Setup CAN listener
        can.Notifier(self.bus, [self])

    def on_message_received(self, msg:can.Message) -> None:
        """Call when CAN message is received.
        
        Implementation of can.listener.Listener callback.
        
        Args:
            msg (can.Message): Incoming CAN message.
        """
        # Extended ID (29 bit)
        if msg.is_extended_id:
            return

        # Standard ID (11 bit)
        func_code, node_id = decode_id(msg.arbitration_id)

        # Not target node ID
        if node_id != self.node_id:
            return

        # print(f"Function Code: {hex(func_code)}")

        # Transmit SDO
        if func_code == FunctionCode.TSDO.value:
            # print("----TSDO Frame----")
            frame = SDOFrame(msg=msg)

            # Index 0
            if frame.index == 0x00:
                # (0,0) Target rotation reached
                if frame.subindex == 0x00:
                    print("Target position reached")
                    self.is_running = False
                # (0,1) Target speed set
                elif frame.subindex == 0x01:
                    print("Target speed set")
                # (0,2) Read current positionfunc_code
                elif frame.subindex == 0x02:
                    self.current_position = frame.data / (2**16-1)*360 - 180
                    print(f"Current postion = {self.current_position:.2f} deg")

    def on_error(self, exc: Exception) -> None:
        """CAN communcation error."""
        return self.on_error(exc)

    def set_target_position(self, position:float):
        """Set bearing target rotation.
        
        Args:
            target (float): Target position in degrees.
        """
        data = (position + 180)/360*(2**16-1)
        print(f"target position: {data}")
        frame = sdo_download_request(self.node_id, 0x00, 0x00, int(data))
        self.bus.send(frame.encode_message())

    def set_target_speed(self, speed:int):
        """Set bearing target speed.
        
        Args:
            speed (int): Target speed ranging from 0 to 255.
        """
        frame = sdo_download_request(self.node_id, 0x00, 0x01, speed)
        self.bus.send(frame.encode_message())
        self.is_running = True
    
    def read_current_position(self):
        """Read current bearing rotation."""
        frame = sdo_download_request(self.node_id, 0x00, 0x02, 0x00)
        self.bus.send(frame.encode_message())

    def set_positional_feedback(self, interval:int):
        """Set breaing rotation feedback interval.

        Args:
            interval (ctypes.c_uint32): Feedback interval in milliseconds.
        """
        frame = sdo_download_request(self.node_id, 0x00, 0x03, interval)
        self.bus.send(frame.encode_message())


def main(args=None):
    """Run when this script is called."""
    bus = can.ThreadSafeBus(
        # interface='seeedstudio', channel='/dev/ttyUSB0', baudrate=2000000, bitrate=500000
        # interface='seeedstudio', channel='COM5', baudrate=2000000, bitrate=500000
        interface='socketcan', channel='can0', bitrate=500000
    )

    time.sleep(0.5)

    driver = BearingDriver(bus, 0x0E)
    time.sleep(0.5)
    driver.set_positional_feedback(200)
    driver.set_target_speed(120)
    time.sleep(0.5)
    driver.set_target_position(5)
    time.sleep(5)
    driver.set_target_speed(120)
    driver.set_target_position(0)
    time.sleep(5)
    driver.set_positional_feedback(0)

    # bus.shutdown()

if __name__ == '__main__':
    main()
