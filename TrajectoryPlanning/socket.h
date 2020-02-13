//套接字头文件
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
	Socket(int port, int fbind = 0); //接收时把flag设为1
	int sendCommand(Robots_Command* command); //发送运动指令
	int sendDebug(Debug_Msgs* msgs); //发送调试信息
	int receive(Vision_DetectionFrame* frame); //接受图像信息
	~Socket() { closesocket(sock); WSACleanup(); }
};
