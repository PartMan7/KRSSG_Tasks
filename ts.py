# import socket programming library 
import socket 


def rminion(sockets,n):
	sum=0
	for i in range (0,n):
		sum=sum+int((sockets[i].recv(1024)).decode('ascii'))
		sockets[i].close()
	m=str(sum)	
	sockets[n].send(m.encode('ascii'))	
	sockets[n].close



def sminion(sockets,n,lst): 

	m=''
	for i in range (0,len(lst)):
		if int(i/int((len(lst)/n)))<n-1:
			m=m+(str(lst[i]))+' '
			if (int((i+1)/int((len(lst)/n)))-int(i/int((len(lst)/n))))==1:
				m=m+'done'
				sockets[int(i/(len(lst)/n))].send(m.encode('ascii'))
				m=''
		else:
			m=m+str(lst[i])+' '
	m=m+'done'		 
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
	print('input the number of minions')
	m=int(input())

	# a forever loop until client wants to exit 
	for i in range (0,m+1): 

		# establish connection with client 
		c, addr = s.accept() 
		print('Connected to :', addr[0], ':', addr[1])
		data=c.recv(1024)

		# takes a string input nd converts it into an array of numbers
		l=data.decode('ascii')
		j=0
		k=l[j]
		while k!='end':
			j=j+1
			if l[j]==' ':
				lst.append(int(k))
				k=''
			else:
				k=k+l[j]

		n=n+1
		sockets.append(c) 

	if(n==m):
		print('given number of minions are connected'	

	sminion(sockets,n,lst)
		
	s.close() 


if __name__ == '__main__': 
	Main() 
