import socket               
import time

s = socket.socket()        
host = '10.27.124.161'# ip of raspberry pi 
port = 8000  

while (1):
             
    result = s.connect((host, port))
    print(s.recv(1024))
    s.close()
    time.sleep(1)
