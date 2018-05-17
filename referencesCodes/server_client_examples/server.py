import socket

s = socket.socket()
host = '10.27.124.161' #ip of raspberry pi

# host = socket.gethostname()
print host

port = 8000
s.bind((host, port))

s.listen(5)
while True:
    c, addr = s.accept()
    print ('Got connection from',addr)
    c.send('Thank you for connecting')
    c.close()

