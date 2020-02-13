#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <string.h>
#include <iostream>
#include "socket.h"
using namespace std;

Socket::Socket(int port, int fbind)
{
	//初始化
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		cout << "WSAStartup失败" << endl;
		return;
	}

	//创建socket
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
	{
		cout << "创建socket失败，错误代码: " << WSAGetLastError() << endl;
		WSACleanup();
		return;
	}
	else
		cout << "socket创建成功" << endl;

	//服务器地址等信息
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);	//端口
	addr.sin_addr.s_addr = inet_addr("127.0.0.1") ;	//地址

	if (fbind) //如果接收则需要绑定本地端口
		if (::bind(sock, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR)
		{
			cout << "绑定失败，错误代码: " << WSAGetLastError() << endl;
			closesocket(sock);
			WSACleanup();
			return;
		}
		else
			cout << "端口绑定成功" << endl;
}

int Socket::sendCommand(Robots_Command* commands)
{
	int buffsize = (*commands).ByteSize();
	char *data = new char[buffsize];
	(*commands).SerializeToArray(data, buffsize);
	if (sendto(sock, data, buffsize, 0, (SOCKADDR*)&addr, sizeof(addr))== SOCKET_ERROR)
	{
		cout << "发送失败，错误代码: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		//cout << "控制指令发送成功" << endl;
		return TRUE;
	}
}

int Socket::sendDebug(Debug_Msgs* msgs)
{
	int buffsize = (*msgs).ByteSize();
	char *data = new char[buffsize];
	(*msgs).SerializeToArray(data, buffsize);
	if (sendto(sock, data, buffsize, 0, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR)
	{
		cout << "发送失败，错误代码: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		//cout << "发送成功" << endl;
		return TRUE;
	}
}

int Socket::receive(Vision_DetectionFrame* frame)
{
	char data[BUFFER_SIZE];
	int fromlen = sizeof(addr);
	if (recvfrom(sock, data, sizeof(data), 0, (SOCKADDR*)&addr, &fromlen) == INVALID_SOCKET)
	{
		cout << "接收失败，错误代码: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		(*frame).ParseFromArray(data, size(data));
		return TRUE;
	}
}
