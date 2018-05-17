import socket
import time

host = 'localhost'
port = 8000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))

for index in xrange(5):
    data = "GET\nSONAR%d\n\n" % index
    print 'send to server: ' + data
    client_socket.send(data)
    while client_socket.recv(2048) != "ack":
        print "waiting for ack"
    print "ack received!"
    time.sleep(1)

#send disconnect message                                                                                                                           
dmsg = "disconnect"
print "Disconnecting"
client_socket.send(dmsg)

client_socket.close()