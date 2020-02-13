#pragma once
#include "socket.h"
#include <vector>

class point
{
	double x, y;
	int parent;
	double cost;

public:
	point()
	{
		x = 0;
		y = 0;
		parent = -1;
	}
	void setx(double x)
	{
		this->x = x;
	}
	void sety(double y)
	{
		this->y = y;
	}
	void setparent(int i)
	{
		parent = i;
	}
	double getx()
	{
		return x;
	}
	double gety()
	{
		return y;
	}
	inline int getparent()
	{
		return parent;
	}
	double getCost()
	{
		return cost;
	}
	void setCost(double cost)
	{
		this->cost = cost;
	}
};

class RRTStar
{
	//障碍物数量
	int cnt = 0;
	//自己的位置
	point self;
	//障碍物中心点位置
	point obstacle[32];
	//点集，边集由parent连接
	std::vector<point> seeds; 

public:
	//rrt主程序
	void rrt_star(point start, point end);
	//读取障碍物并创建地图
	void updateMap(Vision_DetectionFrame frame);
	//读取指定点状态
	inline bool getState(point p);
	//找到点集中距离xrand最近的点
	int findNearestPoint(point p);
	//检查两点连线是否有障碍物
	bool checkObstacle(point *p1, point *p2);
	//检查点是否在地图内
	bool inMap(point p);
	//随机生成点xrand
	point generateRandomNode(point end);
	//检查是否到达目标
	bool checkGoal(point current, point end);
	//根据xrand的方向生成xnew
	point steer(point xnearest, point xrand);
	//返回两点距离
	inline double distance(point p1, point p2);
	//路径平滑
	void smooth();
	//返回自己的信息
	point getSelf()
	{
		return self;
	};
	//打印路径
	void printPath(std::vector<double>* xpos, std::vector<double>* ypos);
	Debug_Msg* drawLine(Debug_Msg* msg, point p1, point p2, bool forward = 1, bool backward = 0);
};