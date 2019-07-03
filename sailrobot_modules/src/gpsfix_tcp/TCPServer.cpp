#include "gpsfix_tcp/TCPServer.h" 

string TCPServer::Message;

void* TCPServer::Task(void *arg)
{
	int n;
	int newsockfd = (long)arg;
	char msg[MAXPACKETSIZE];
	pthread_detach(pthread_self());
	while(1)
	{
		n=recv(newsockfd,msg,MAXPACKETSIZE,0);
		if(n==0)
		{
		   close(newsockfd);
		   break;
		}
		msg[n]=0;
		//send(newsockfd,msg,n,0);
		Message = string(msg);
        }
	return 0;
}

void TCPServer::setup(int port)
{
	sockfd=socket(AF_INET,SOCK_STREAM,0);
 	memset(&serverAddress,0,sizeof(serverAddress));
	serverAddress.sin_family=AF_INET;
	serverAddress.sin_addr.s_addr=htonl(INADDR_ANY);
	serverAddress.sin_port=htons(port);
	bind(sockfd,(struct sockaddr *)&serverAddress, sizeof(serverAddress));
 	listen(sockfd,5);
}

int TCPServer::acceptsock(int timeout){
	socklen_t sosize = sizeof(clientAddress);
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(sockfd, &rfds);

	struct timeval tv;
	tv.tv_sec = (long)timeout;
	tv.tv_usec = 0;

	int res = select(sockfd+1, &rfds, (fd_set *)0, (fd_set *) 0, &tv);
	if(res > 0)
		return accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
	else
		return -1;
}

string TCPServer::receive()
{
	string str;
	while(1)
	{
		socklen_t sosize  = sizeof(clientAddress);
		newsockfd = accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
		str = inet_ntoa(clientAddress.sin_addr);
		pthread_create(&serverThread,NULL,&Task,(void *)newsockfd);
	}
	return str;
}

string TCPServer::getMessage()
{
	return Message;
}

void TCPServer::Send(string msg)
{
	send(newsockfd,msg.c_str(),msg.length(),0);
}

void TCPServer::clean()
{
	Message = "";
	memset(msg, 0, MAXPACKETSIZE);
}

void TCPServer::detach()
{
	std::cout << "Closing Sockets" << std::endl;
	if(sockfd > 0){
		int err = 1;
   		socklen_t len = sizeof err;
   		if (-1 == getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char *)&err, &len))
      			std::cerr << "getSO_ERROR" << std::endl;
   		if (err)
      			errno = err;              // set errno to the socket SO_ERROR
	}
	if(newsockfd > 0){
		int err = 1;
   		socklen_t len = sizeof err;
   		if (-1 == getsockopt(newsockfd, SOL_SOCKET, SO_ERROR, (char *)&err, &len))
      			std::cerr << "getSO_ERROR" << std::endl;
   		if (err)
      			errno = err;              // set errno to the socket SO_ERROR
	}
	close(sockfd);
	close(newsockfd);
} 
