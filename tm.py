
# Import socket module 
import socket 

def sum(lst,s):
    n=0
    for i in range (0,len(lst)):
        n=n+lst[i]
    m=str(n)    
    s.send(m.encode('ascii'))   



  
def Main(): 
    # local host IP '127.0.0.1' 
    host = '127.0.0.1'
  
    # Define the port on which you want to connect 
    port = 12345
  
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
  
    # connect to server on local computer 
    s.connect((host,port)) 
    lst=[]
    m='end'
    s.send(m.encode('ascii'))
  
    # messaga received from server 
    data = s.recv(1024) 
    l=data.decode('ascii')
    j=0
    k=l[j]
    while k!='done':
        j=j+1
        if l[j]==' ':
            lst.append(int(k))
            k=''
        else:
            k=k+l[j]

    print('Received from the server :',lst) 
    sum(lst,s)    

    s.close() 
  
if __name__ == '__main__': 
    Main() 
