# import socket programming library 
import socket 


def rminion(sockets,n):
	sum=0
	for i in range (0,n):
		sum=sum+int((c[i].recv(1024)).decode('ascii'))
		sockets[i].close()
	sum=str(sum)	
	sockets[n].send(sum,encode('ascii'))	
	sockets[n].close



def sminion(sockets,n,lst): 

	for i in range (0,len(lst)):
		if (len(lst)-i)>=2*n:
			m=str(lst[i]) 
			sockets[int(i/n)].send(m.encode('ascii'))
			if (int((i+1)/n)-int(i/n))==1:
				m='done'
				sockets[int(i/n)].send(m.encode('ascii'))
		else:
			sockets[n-1].send(lst[i].encode('ascii'))
			sockets[n-1].send(m.encode('ascii'))
	rminion(sockets,n)		



def Main(): 
	host = "" 

	# reverse a port on your computer 
	# in our case it is 12345 but it 
	# can be anything 
	port = 12345
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	s.bind((host, port)) 
	print("socket binded to port", port) 

	# put the socket into listening mode 
	s.listen(10) 
	print("socket is listening")
	n=-1 
	sockets=[]
	lst=[]

	# a forever loop until client wants to exit 
	while True: 

		# establish connection with client 
		c, addr = s.accept() 
		print('Connected to :', addr[0], ':', addr[1])
		data=c.recv(1024)

		if not data:
			break
		if len(data.decode('ascii'))>1:
			lst=int(data.decode('ascii'))
			print('array received')

		n=n+1
		sockets.append(c) 
		
	sminion(sockets,n,lst)
		
	s.close() 


if __name__ == '__main__': 
	Main() 
