//�׽���ͷ�ļ�
#pragma once
#include <WinSock2.h>
#include "vision_detection.pb.h"
#include "zss_cmd.pb.h"
#include "zss_debug.pb.h"
#pragma comment(lib,"ws2_32.lib")
constexpr auto BUFFER_SIZE = 4096;

class Socket
{
	SOCKET sock;
	SOCKADDR_IN addr;

public:
	Socket(int port, int fbind = 0); //����ʱ��flag��Ϊ1
	int sendCommand(Robots_Command* command); //�����˶�ָ��
	int sendDebug(Debug_Msgs* msgs); //���͵�����Ϣ
	int receive(Vision_DetectionFrame* frame); //����ͼ����Ϣ
	~Socket() { closesocket(sock); WSACleanup(); }
};
