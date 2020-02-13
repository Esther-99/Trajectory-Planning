#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <string.h>
#include <iostream>
#include "socket.h"
using namespace std;

Socket::Socket(int port, int fbind)
{
	//��ʼ��
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		cout << "WSAStartupʧ��" << endl;
		return;
	}

	//����socket
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
	{
		cout << "����socketʧ�ܣ��������: " << WSAGetLastError() << endl;
		WSACleanup();
		return;
	}
	else
		cout << "socket�����ɹ�" << endl;

	//��������ַ����Ϣ
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);	//�˿�
	addr.sin_addr.s_addr = inet_addr("127.0.0.1") ;	//��ַ

	if (fbind) //�����������Ҫ�󶨱��ض˿�
		if (::bind(sock, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR)
		{
			cout << "��ʧ�ܣ��������: " << WSAGetLastError() << endl;
			closesocket(sock);
			WSACleanup();
			return;
		}
		else
			cout << "�˿ڰ󶨳ɹ�" << endl;
}

int Socket::sendCommand(Robots_Command* commands)
{
	int buffsize = (*commands).ByteSize();
	char *data = new char[buffsize];
	(*commands).SerializeToArray(data, buffsize);
	if (sendto(sock, data, buffsize, 0, (SOCKADDR*)&addr, sizeof(addr))== SOCKET_ERROR)
	{
		cout << "����ʧ�ܣ��������: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		//cout << "����ָ��ͳɹ�" << endl;
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
		cout << "����ʧ�ܣ��������: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		//cout << "���ͳɹ�" << endl;
		return TRUE;
	}
}

int Socket::receive(Vision_DetectionFrame* frame)
{
	char data[BUFFER_SIZE];
	int fromlen = sizeof(addr);
	if (recvfrom(sock, data, sizeof(data), 0, (SOCKADDR*)&addr, &fromlen) == INVALID_SOCKET)
	{
		cout << "����ʧ�ܣ��������: " << WSAGetLastError() << endl;
		return FALSE;
	}
	else
	{
		(*frame).ParseFromArray(data, size(data));
		return TRUE;
	}
}
