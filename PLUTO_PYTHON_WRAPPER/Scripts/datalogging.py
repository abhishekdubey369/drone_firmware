import socket

# Define states for data handling
IDLE = 0
HEADER_START = 1
HEADER_M = 2
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6
HEADER_D = 7
HEADER_D_SIZE = 8

class DataHandler:
    def __init__(self):
        self.c_state = IDLE
        self.err_rcvd = False
        self.offset = 0
        self.dataSize = 0
        self.checksum = 0
        self.debugString = []
        self.receive_buffer = []

    def read(self, readData):
        self.receive_buffer.extend(readData)
        data = self.receive_buffer[:]
        self.receive_buffer.clear()
        self.processData(data)

    def processData(self, data):
        for byte in data:
            if self.c_state == IDLE:
                self.c_state = HEADER_START if byte == 36 else IDLE
            elif self.c_state == HEADER_START:
                if byte == 77:
                    self.c_state = HEADER_M
                elif byte == 68:
                    self.c_state = HEADER_D
                else:
                    self.c_state = IDLE
            elif self.c_state == HEADER_M:
                if byte == 62:
                    self.c_state = HEADER_ARROW
                elif byte == 33:
                    self.c_state = HEADER_ERR
                else:
                    self.c_state = IDLE
            elif self.c_state == HEADER_ARROW or self.c_state == HEADER_ERR:
                self.err_rcvd = (self.c_state == HEADER_ERR)
                self.dataSize = byte & 0xFF
                self.offset = 0
                self.checksum = 0
                self.checksum ^= (byte & 0xFF)
                self.c_state = HEADER_SIZE
            elif self.c_state == HEADER_SIZE:
                self.checksum ^= (byte & 0xFF)
                self.c_state = HEADER_CMD
            elif self.c_state == HEADER_CMD and self.offset < self.dataSize:
                self.checksum ^= (byte & 0xFF)
                self.offset += 1
            elif self.c_state == HEADER_CMD and self.offset >= self.dataSize:
                if (self.checksum & 0xFF) == (byte & 0xFF):
                    if not self.err_rcvd:
                        pass
                self.c_state = IDLE
            elif self.c_state == HEADER_D:
                self.dataSize = byte & 0xFF
                self.offset = 0
                self.checksum = 0
                self.checksum ^= (byte & 0xFF)
                self.c_state = HEADER_D_SIZE
            elif self.c_state == HEADER_D_SIZE and self.offset < self.dataSize:
                self.checksum ^= (byte & 0xFF)
                self.debugString.append(byte)
                self.offset += 1
            elif self.c_state == HEADER_D_SIZE and self.offset >= self.dataSize:
                if (self.checksum & 0xFF) == (byte & 0xFF):
                    if not self.err_rcvd:
                        if self.debugString and chr(self.debugString[0]) != '~':
                            received_data = ''.join(chr(b) for b in self.debugString if 32 <= b <= 126)
                            print(f'{received_data}')
                self.c_state = IDLE
                self.debugString.clear()

def main():
    host = '192.168.4.1'  # Replace with your target IP address
    port = 23  # Replace with your target port

    data_handler = DataHandler()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print("connected")

        try:
            while True:
                data = sock.recv(1024)
                if not data:
                    break
                data_handler.read(data)
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
