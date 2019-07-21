import sys
import bluetera_messages_pb2 as bltr
from google.protobuf.internal.decoder import _DecodeVarint32

def read_log_file(f):
    data = f.read()
    next_pos, pos = 0, 0
    samples = []

    while pos < len(data):
        msg = bltr.DownlinkMessage()
        next_pos, pos = _DecodeVarint32(data, pos)
        msg.ParseFromString(data[pos:pos + next_pos])
        pos += next_pos
        samples.append(msg)
    
    return samples

if __name__ == "__main__":
    arg_len = len(sys.argv)

    if arg_len > 1:
        filename = sys.argv[1]
    else:
        print("missing filename")
        print("usage example: log_extractor LOG1")
        sys.exit()

    with open(filename, "rb") as f:
        messages = read_log_file(f)
    
    with open("{0}.csv".format(filename), "w") as f:
        for msg in messages:
            if msg.WhichOneof("payload") == "quaternion":
                f.write("{0},{1},{2},{3}\n".format(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w))
            elif msg.WhichOneof("payload") == "accelerometer":
                f.write("{0},{1},{2}\n".format(msg.accelerometer.x, msg.accelerometer.y, msg.accelerometer.z))