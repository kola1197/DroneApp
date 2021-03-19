from PXConnector import PXConnector
import socket
import sys


class Main:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.PX = PXConnector()
        self.__stop = False

    def stop(self):
        self.__stop = True

    def start(self):
        self.CreateServer()
        # while not self.__stop:
        #   try:
        #       self.CreateServer()
        #   finally:
        #       pass

    def reactToData(self, data):
        s = data.replace('_', ' ')
        ch = [int(x) for x in s.split()]
        print("got data: " + str(ch))
        self.PX.vehicle.channels.overrides = {'1': ch[0], '2': ch[1], '3': ch[2], '4': ch[3]}

    def CreateServer(self):
        #file = open("Server.txt", "w")
        #file.write("I AM ALIVE!!!")
        #file.close()
        self.PX.Connect()
        if (self.PX.connected):
            server_address = ('localhost', 60239)
            print(sys.stdout, 'starting up on %s port %s' % server_address)
            self.sock.bind(server_address)
            self.sock.listen(1)
            # while True:
            # Wait for a connection
            print('waiting for a connection')
            connection, client_address = self.sock.accept()
            try:
                print(sys.stdout, 'connection from', client_address)
                # Receive the data in small chunks and retransmit it
                while True:
                    data = connection.recv(26).decode()
                    if data:
                        self.reactToData(data)
                        # print >> sys.stderr, 'sending data back to the client'
                        # connection.sendall(data)
                    else:
                        print(sys.stderr, 'no more data from', client_address)
                        break
            finally:
                # Clean up the connection
                connection.close()

    def reactToMessage(self, msg):
        print(msg)


if __name__ == '__main__':
    M = Main()
    M.start()
