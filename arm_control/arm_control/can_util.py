"""
CANopen utility library

Written by:
    1. Kean Hao
Last Updated: 6 Jan 2023
"""

from enum import Enum
import can

class FunctionCode(Enum):
    """Enum of CANopen function code"""
    NMT = 0x0 #Network Management
    SYNC = 0x1 #Synchronization Object (Node ID: 0)
    TIME = 0x2 #Time Stamp
    EMCY = 0x1 #Emergency (Node ID: 1-126)

    #TPDO -> Transmit Process Data Object (TX)
    #RPDO -> Receive Process Data Object (RX)
    TPDO1 = 0x3
    RPDO1 = 0x4
    TPDO2 = 0x5
    RPDO2 = 0x6
    TPDO3 = 0x7
    RPDO3 = 0x8
    TPDO4 = 0x9
    RPDO4 = 0xA

    #TSDO -> Transmit Service Data Object (TX)
    #RSDO -> Receive Service Data Object (RX)
    TSDO = 0xB
    RSDO = 0xC

    NMT_ERROR = 0xE #NMT Error

def decode_id(id: int):
    """Decode CAN ID into CANopen function code and node ID"""
    return ((id >> 7) & 0xF, id & 0x7F)

class SDOFrame:
    """CANopen SDO Frame class"""

    class CS(Enum):
        """Enum of SDO Command Specifier"""
        DOWNLOAD_REQUEST_4 = 0x23 #Client -> Server: SDO Download Request (4 bytes)
        DOWNLOAD_REPONSE = 0x60 #Server -> Client: SDO Download Response
        UPLOAD_REQUEST = 0x40 #Client -> Server: SDO Upload Request
        UPLOAD_RESPONSE_4 = 0x43 #Server -> Client: SDO Upload Response
        ABORT_DOMAIN_TRANSFER = 0x80 #Error

    def __init__(self, node_id=0x00, is_receive_sdo=True, cs=0x00, index=0x0000, subindex=0x00,
        data=0x00000000, msg:can.Message=None) -> None:
        #Encode from input parameters
        if msg is None:
            if is_receive_sdo:
                self.func_code = FunctionCode.RSDO #Client -> Server
            else:
                self.func_code = FunctionCode.TSDO #Server -> Client

            self.node_id = node_id
            self.cs = cs
            self.index = index
            self.subindex = subindex
            self.data = data

        #Decode from CAN message
        else:
            self.func_code, self.node_id = decode_id(msg.arbitration_id)
            self.cs = msg.data[0]
            self.index = (msg.data[2] << 8) | msg.data[1]
            self.subindex = msg.data[3]
            self.data = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4]
        
        # print("NodeID\tCS\tIndex\tSubindex\tData")
        # print(f"{hex(self.node_id)}\t{hex(self.cs)}\t{hex(self.index)}\t{hex(self.subindex)}\t{hex(self.data)}")

    def encode_message(self) -> can.Message:
        """Encode into CAN frame"""
        # print(f"Index: {hex(self.index)}")
        # print(f"Decode data 0: {hex(self.data&0xFF)}")
        # print(f"Decode data 1: {hex((self.data>>8)&0xFF)}")
        # print(f"Decode data 2: {hex((self.data>>16)&0xFF)}")
        # print(f"Decode data 3: {hex((self.data>>24)&0xFF)}")
        return can.Message(
            arbitration_id = (self.func_code.value<<7) | self.node_id,
            data = [
                self.cs, #Commad Specifier
                self.index&0xFF, (self.index>>8)&0xFF, #Index
                self.subindex, #Sub index
                self.data&0xFF, (self.data>>8)&0xFF, (self.data>>16)&0xFF, (self.data>>24)&0xFF #Data
                ],
            is_extended_id=False,
            dlc=8
        )

class PDOFrame:
    """CANopen PDO Frame class"""
    def __init__(self, function_code:FunctionCode=None, node_id=0x00, data=0x00000000,
        msg:can.Message=None) -> None:
        #Encode from input parameters
        if msg is None:
            self.func_code = function_code
            self.node_id = node_id
            self.data = data

        #Decode from CAN message
        else:
            self.func_code, self.node_id = decode_id(msg.arbitration_id)
            self.data = msg.data

        # print("NodeID\tData0\tData1\tData2\tData3\tData4\tData5\tData6\tData7")
        # print(f"{hex(self.node_id)}\t{hex(self.data[0])}\t{hex(self.data[1])}\t{hex(self.data[2])}\t{hex(self.data[3])}\t{hex(self.data[4])}\t{hex(self.data[5])}\t{hex(self.data[6])}\t{hex(self.data[7])}\t")

    def encode_message(self) -> can.Message:
        """Encode into CAN frame"""
        return can.Message(
            arbitration_id = (self.func_code.value<<7) | self.node_id,
            data = self.data,
            is_extended_id=False,
        )

def sdo_upload_request(node_id, index, subindex) -> SDOFrame:
    """(RSDO) Client -> Server: Read Request"""
    return SDOFrame(node_id, True, SDOFrame.CS.UPLOAD_REQUEST.value, index, subindex, 0x00000000)

def sdo_upload_response(node_id, index, subindex, data) -> SDOFrame:
    """(TSDO) Server -> Client: Read Response"""
    return SDOFrame(node_id, False, SDOFrame.CS.UPLOAD_RESPONSE_4.value, index, subindex, data)

def sdo_download_request(node_id, index, subindex, data) -> SDOFrame:
    """(RSDO) Client -> Server: Write Request"""
    return SDOFrame(node_id, True, SDOFrame.CS.DOWNLOAD_REQUEST_4.value, index, subindex, data)

def sdo_download_response(node_id, index, subindex) -> SDOFrame:
    """(TSDO) Client -> Server: Write Response"""
    return SDOFrame(node_id, False, SDOFrame.CS.DOWNLOAD_REPONSE.value, index, subindex, 0x00000000)
