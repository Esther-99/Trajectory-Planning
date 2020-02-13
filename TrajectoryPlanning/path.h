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
	//�ϰ�������
	int cnt = 0;
	//�Լ���λ��
	point self;
	//�ϰ������ĵ�λ��
	point obstacle[32];
	//�㼯���߼���parent����
	std::vector<point> seeds; 

public:
	//rrt������
	void rrt_star(point start, point end);
	//��ȡ�ϰ��ﲢ������ͼ
	void updateMap(Vision_DetectionFrame frame);
	//��ȡָ����״̬
	inline bool getState(point p);
	//�ҵ��㼯�о���xrand����ĵ�
	int findNearestPoint(point p);
	//������������Ƿ����ϰ���
	bool checkObstacle(point *p1, point *p2);
	//�����Ƿ��ڵ�ͼ��
	bool inMap(point p);
	//������ɵ�xrand
	point generateRandomNode(point end);
	//����Ƿ񵽴�Ŀ��
	bool checkGoal(point current, point end);
	//����xrand�ķ�������xnew
	point steer(point xnearest, point xrand);
	//�����������
	inline double distance(point p1, point p2);
	//·��ƽ��
	void smooth();
	//�����Լ�����Ϣ
	point getSelf()
	{
		return self;
	};
	//��ӡ·��
	void printPath(std::vector<double>* xpos, std::vector<double>* ypos);
	Debug_Msg* drawLine(Debug_Msg* msg, point p1, point p2, bool forward = 1, bool backward = 0);
};