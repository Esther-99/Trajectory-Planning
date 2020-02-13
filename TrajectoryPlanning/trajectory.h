#pragma once
#include <vector>
#include "socket.h"

class Trajectory
{
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> ori; //�����Ƕ�
	int cnt;
	Socket* commandSock;
	Robots_Command commands;
	Socket* frameSock;	
	Vision_DetectionFrame frame;

public:
	Trajectory(Socket* commandSock, Socket* frameSock);
	//�޸�·����
	void updateNodes(std::vector<double>* x, std::vector<double>* y);
	//���й켣�滮
	void plan();
	//��ʼ��Ƕ��޸�
	double rotate();
	//ǰ���˶�
	void forward(int flag, double* lastw, double* lastv);
	//ת�۵�
	//void transit();
};