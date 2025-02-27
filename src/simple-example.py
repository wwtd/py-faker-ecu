import can
import time
import math
import threading

SINGLE_FRAME = 0x0           # Single Frame
FIRST_FRAME  = 0x1           # First Frame
CONSECUTIVE_FRAME = 0x2      # Continute Frame
FLOW_CONTROL = 0x3           # Flow Control Frame

CANTP_TX_id = 0xE1
CANTP_RX_id = 0xE4

CAN_BUS_ID = 'vcan0'

CONTROL_FLOW_TIMEOUT = 100 #ms

def iso_tp_send(bus, tx_id, rx_id, data):
    if(len(data) <=7):
        pci = pci = (SINGLE_FRAME << 4) | len(data)
        frame_data = bytes([pci]) + data
        frame_data = frame_data.ljust(8, b'\x00')
        msg = can.Message(arbitration_id=tx_id, data=frame_data, is_extended_id=False)
        bus.send(msg)
        print("Sent Single Frame:", msg)
    else:
        total_length = len(data)
        pci_first = (FIRST_FRAME << 4) | ((total_length >> 8) & 0xF)
        pci_second = total_length & 0xFF
        first_frame_data = bytes([pci_first, pci_second]) + data[:6]
        first_frame_data = first_frame_data.ljust(8, b'\x00')
        msg = can.Message(arbitration_id=tx_id, data=first_frame_data, is_extended_id=False)
        bus.send(msg)
        print("Sent First Frame:", msg)

        flow_control_received = False
        fc_timeout = (int(time.time() * 1000)) + CONTROL_FLOW_TIMEOUT
        while(int(time.time() * 1000) < fc_timeout):
            fc_msg = bus.recv(timeout=0.001)
            if fc_msg and fc_msg.arbitration_id == rx_id:
                if (fc_msg.data[0] >> 4) == FLOW_CONTROL:
                    flow_control_received = True
                    block_size = fc_msg.data[1]
                    stmin = fc_msg.data[2]
                    print("Received Flow Control: block_size={}, stmin={}".format(block_size, stmin))
                    break

        if not flow_control_received:
            print("No Flow Control received, aborting transmission.")
            return

        remaining = data[6:]
        num_frames = math.ceil(len(remaining) / 7)
        seq = 1
        for i in range(num_frames):
            frame_payload = remaining[i*7:(i+1)*7]
            pci = (CONSECUTIVE_FRAME << 4) | (seq & 0xF)
            cf_data = bytes([pci]) + frame_payload
            cf_data = cf_data.ljust(8, b'\x00')
            msg = can.Message(arbitration_id=tx_id, data=cf_data, is_extended_id=False)
            bus.send(msg)
            print("Sent Consecutive Frame {}:".format(seq), msg)
            seq = (seq + 1) & 0xF
            #stmin
            time.sleep(0.001)

def iso_tp_receive(bus, tx_id, rx_id):
    print(f"Waiting for ISO-TP message, tx_id is {tx_id}, rx_id is {rx_id}")
    assembled_data = b''
    total_length = None
    expected_seq = 1

    while True:
        msg = bus.recv(timeout = 0.001)
        if msg is None:
            continue
        if msg.arbitration_id != rx_id:
            continue
        pci = msg.data[0]
        frame_type = pci >> 4
        if frame_type == SINGLE_FRAME:
            sf_len = pci & 0x0F
            assembled_data = msg.data[1:1+sf_len]
            print("Received Single Frame:", msg)
            break
        elif frame_type == FIRST_FRAME:
            total_length = ((pci & 0x0F) << 8) | msg.data[1]
            assembled_data = msg.data[2:8]
            print("Received First Frame:", msg)
            fc = bytes([ (FLOW_CONTROL << 4) | 0x0, 0x00, 0x00 ])
            fc = fc.ljust(8, b'\x00')
            fc_msg = can.Message(arbitration_id=tx_id, data=fc, is_extended_id=False)
            bus.send(fc_msg)
            print("Sent Flow Control Frame:", fc_msg)
        elif frame_type == CONSECUTIVE_FRAME:
            seq = pci & 0x0F
            if seq != expected_seq:
                print("Sequence number mismatch: expected {}, got {}".format(expected_seq, seq))
                break
            assembled_data += msg.data[1:8]
            expected_seq = (expected_seq + 1) & 0xF
            if total_length is not None and len(assembled_data) >= total_length:
                assembled_data = assembled_data[:total_length]
                print("Received complete ISO-TP message.")
                # print("msg: ", assembled_data.hex())
                return assembled_data
                break
        else:
            print("Unknown frame type:", frame_type)


def main():
    bus = can.interface.Bus(channel=CAN_BUS_ID, bustype='socketcan')
    # message =  bytes([0x01, 0x02, 0x03, 0x04])
    # iso_tp_send(bus, 0x72, 0x72, message)

    # message =  bytes([0x01, 0x02, 0x03, 0x04,0x01, 0x02, 0x03, 0x04,0x01, 0x02, 0x03, 0x04,0x01, 0x02, 0x03, 0x04])
    # iso_tp_send(bus, 0xE1, 0xE4, message)

    # iso_tp_receive(bus, 0xE1, 0xE4)
    while True:
        data = iso_tp_receive(bus, 0xE1, 0xE4)
        print("data: ", data.hex())
    bus.shutdown()


if __name__ == '__main__':
    main()
