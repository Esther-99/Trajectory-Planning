#pragma once
#include <vector>
#include "socket.h"

class Trajectory
{
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> ori; //期望角度
	int cnt;
	Socket* commandSock;
	Robots_Command commands;
	Socket* frameSock;	
	Vision_DetectionFrame frame;

public:
	Trajectory(Socket* commandSock, Socket* frameSock);
	//修改路径点
	void updateNodes(std::vector<double>* x, std::vector<double>* y);
	//进行轨迹规划
	void plan();
	//初始点角度修改
	double rotate();
	//前向运动
	void forward(int flag, double* lastw, double* lastv);
	//转折点
	//void transit();
};