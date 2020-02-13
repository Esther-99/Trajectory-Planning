#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string.h>
#include <time.h>
#include "path.h"
using namespace std;
constexpr auto MAXCYCLE = 10000; //最大循环次数
constexpr auto STEP = 10; //步长
constexpr auto RADIUS = 25; //Xnear半径范围
constexpr auto OBSTACLE_R = 27; //障碍物扩张半径
constexpr auto MAX_SMOOTH = 10; //平滑连线次数限制

void RRTStar::rrt_star(point start, point end)
{
	point xrand;
	int xnearest, xmin, current;
	int count;
	std::vector<int> Xnear;
	std::vector<double> dist;
	double cost, cmin;

	start.setCost(0); //起始点路径消耗为0
	seeds.push_back(start); //起始点加入点集
	int cycles = 0;
	while (cycles < MAXCYCLE)
	{
		Xnear.clear();
		dist.clear();
		count = 0;
		point xnew;
		//自由空间随机取点
		xrand = generateRandomNode(end);
		//找到图中与随机点最近的点
		xnearest = findNearestPoint(xrand);
		//生成新的点
		xnew = steer(seeds[xnearest], xrand);
		if (!checkObstacle(&seeds[xnearest], &xnew) && inMap(xnew)) //无障碍物
		{
			cycles++;
		}
		else //无效的xnew，重新找点
		{
			continue;
		}

		//RRT*第一步，迭代寻找最小路径消耗的点
		xmin = xnearest;
		cmin = seeds[xnearest].getCost() + distance(seeds[xnearest], xnew);
		for (int i = 0; i < seeds.size(); i++)
		{
			double dis = distance(seeds[i], xnew);
			if (dis < RADIUS) //原来点集中的点与xnew距离小于一定半径，检查xnew要不要换爸爸
			{
				if (!checkObstacle(&xnew, &seeds[i])) //无碰撞
				{
					Xnear.push_back(i); //记录XNear集合
					dist.push_back(dis);//记录Xnear集合中的点与xnew的距离
					count++;
					cost = dis + seeds[i].getCost();
					if (cost < cmin) //若路径消耗更小，则改变父节点
					{
						xmin = i;
						cmin = cost;
					}
				}
			}
		}
		xnew.setCost(cmin); //xnew路径耗散
		xnew.setparent(xmin); //把边连上
		//cout << xnew.getx()<<", "<<xnew.gety()<<"'s parent: " << seeds[xnew.getparent()].getx() <<", "<< seeds[xnew.getparent()].gety() << endl;
		seeds.push_back(xnew); //xnew加入点集


		//RRT*第二步，重新连线
		int last = seeds.size() - 1;
		for (int i = 0; i < count; i++) 
		{
			current = Xnear[i];
			double dis = dist[i];
			double newcost = xnew.getCost()+dis;
			if (newcost < seeds[current].getCost())
			{
				//rewire
				seeds[current].setparent(last);
				seeds[current].setCost(newcost);
				//cout << seeds[current].getx() << ", " << seeds[current].gety() << "-> parent changed to ->" << xnew.getx() << ", " << xnew.gety() << endl;
			}
		}

		//检查是否到达目标
		if (checkGoal(xnew, end))
		{
			cout << "Goal found. Total points: " << seeds.size() << endl;
			break;
		}
		//cout << cycles << endl;
	}

	return;
}

int RRTStar::findNearestPoint(point p)
{
	double dis, mindis=2000;
	int minp = 0;
	int i;
	for (i = 0; i < seeds.size(); i++)
	{
		dis = distance(p, seeds[i]);
		if (dis < mindis)
		{
			minp = i;
			mindis = dis;
		}
	}
	//cout << "Nearest point found: " << seeds[minp].getx() << " , " << seeds[minp].gety() << endl;
	return minp;
}

bool RRTStar::checkObstacle(point *p1, point *p2)
{
	point p;
	p.setx(p1->getx());
	p.sety(p1->gety());
	double deltax = (p2->getx() - p1->getx()) / 100;
	double deltay = (p2->gety() - p1->gety()) / 100;
	for (int i = 1; i <= 100; i++)
	{
		p.setx(p.getx() + deltax);
		p.sety(p.gety() + deltay);
		if (getState(p) == 1)
		{
			//cout << "Obstacle: " << p1.getx() << "," << p1.gety() << " and " << p2.getx() << "," << p2.gety() << endl;
			return TRUE;
		}
	}
	//cout << "Free: " << p1.getx() << "," << p1.gety() << "and" << p2.getx() << "," << p2.gety() << endl;
	return FALSE;
}

bool RRTStar::inMap(point p)
{
	double x = p.getx();
	double y = p.gety();
	if (x > -275 && x < 275 && y<200 && y>-200)
		return TRUE;
	else
		return FALSE;
}

point RRTStar::generateRandomNode(point end)
{
	point p;
	int N = 100;
	double poss;

	srand((unsigned)clock()); //用时间做种子
	poss = (rand() % N) / (double)N;
	if (poss > 0.1) //按概率选取随机点或终点作为xrand
	{
		//地图为600*450，边界处不能生成点，实际使用范围550*400
		int randx = rand() % 500 - 250;
		int randy = rand() % 450 - 225;
		p.setx(randx);
		p.sety(randy);
	}
	else
	{
		p.setx(end.getx());
		p.sety(end.gety());
	}
	
	//cout << "Random point generated: " << p.getx() << "," << p.gety() << endl;

	return p;
}

point RRTStar::steer(point xnearest, point xrand)
{
	point p;
	double dis_x_rand = xrand.getx() - xnearest.getx();
	double dis_y_rand = xrand.gety() - xnearest.gety();
	double length = sqrt(pow(dis_x_rand, 2) + pow(dis_y_rand, 2));
	//double rate_rand = sqrt(pow(STEP, 2) / (pow(dis_x_rand, 2) + pow(dis_y_rand, 2)));
	//以线性的步长/直线距离为扩展率，离得越近“拉力”越大
	double forward_x = xnearest.getx() + STEP * dis_x_rand / length;
	double forward_y = xnearest.gety() + STEP * dis_y_rand / length;
	p.setx(forward_x);
	p.sety(forward_y);
	p.setCost(5000);
	//把检验是否撞墙交给主程序完成
	//cout << "New point generated: " << p.getx() << "," << p.gety() << endl;
	return p;
}

bool RRTStar::checkGoal(point current, point end)
{
	double dis = distance(current, end);
	if (dis < 0.5*STEP)
		return TRUE;
	else
		return FALSE;
}

inline double RRTStar::distance(point p1, point p2)
{
	double x1 = p1.getx();
	double x2 = p2.getx();
	double y1 = p1.gety();
	double y2 = p2.gety();
	double dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	return dis;
}

void RRTStar::updateMap(Vision_DetectionFrame frame)
{
	double x, y;
	int i = 0;

	cnt = 0;
	seeds.clear();
	//自己是蓝车0号
	x = frame.robots_blue(i).x() / 10;
	y = frame.robots_blue(i).y() / 10;
	self.setx(x);
	self.sety(y);
	//创建障碍物中心点信息
	for (i = 0; i < frame.robots_blue_size() ; i++)
	{
		x = frame.robots_blue(i).x() / 10;
		y = frame.robots_blue(i).y() / 10;
		obstacle[cnt].setx(x);
		obstacle[cnt].sety(y);
		cnt++;
	}
	for (i = 0; i < frame.robots_yellow_size(); i++)
	{
		x = frame.robots_yellow(i).x() / 10;
		y = frame.robots_yellow(i).y() / 10;
		obstacle[cnt].setx(x);
		obstacle[cnt].sety(y);
		cnt++;
	}
	cout << "Map updated" << endl;
}

inline bool RRTStar::getState(point p)
{
	double dis;
	//注意轨迹规划时自己不是障碍物，路径规划可以从0开始
	for (int i = 1; i < cnt; i++)
	{
		dis = distance(p, obstacle[i]);
		if (dis < OBSTACLE_R)
		{
			//cout << "state: obstacle" << endl;
			return 1;
		}
	}
	//cout << "state: free" << endl;
	return 0;
}

void RRTStar::smooth()
{
	int son = seeds.size() - 1;
	int count = 0; //避免出现太长的线
	while (seeds[son].getparent() != -1)
	{
		int father = seeds[son].getparent();
		//grandfather存在且连线无碰
		while (seeds[father].getparent() != -1 && !checkObstacle(&seeds[son], &seeds[seeds[father].getparent()]))
		{
			father = seeds[father].getparent();
			count++;
			if (count > MAX_SMOOTH)
			{
				break;
			}
		}
		seeds[son].setparent(father);
		son = father;
		count = 0;
	}
	cout << "Path smoothed" << endl;
}

void RRTStar::printPath(std::vector<double>* xpos, std::vector<double>* ypos)
{
	int current = seeds.size() - 1;
	int prev;
	Socket debugSock(20001, 0);	//创建socket用于发送debug指令
	Debug_Msgs msgs;
	//清空上一次的路径信息
	xpos->clear();
	ypos->clear();

	while (seeds[current].getparent() != -1)
	{
		//保存有用的路径点
		xpos->insert(xpos->begin(), seeds[current].getx());
		ypos->insert(ypos->begin(), seeds[current].gety());
		//cout << seeds[index].getx() << ", " << seeds[index].gety() << endl;
		prev = seeds[current].getparent();
		Debug_Msg* msg = msgs.add_msgs();
		msg = drawLine(msg, seeds[current], seeds[prev]);
		current = prev;
	}
	xpos->insert(xpos->begin(), seeds[current].getx());
	ypos->insert(ypos->begin(), seeds[current].gety());
	//发送debug信息
	debugSock.sendDebug(&msgs);
	cout << "Path printed" << endl;
	//cout << seeds[index].getx() << ", " << seeds[index].gety() << endl;
}

Debug_Msg* RRTStar::drawLine(Debug_Msg* msg, point st, point end, bool forward, bool backward)
{
	msg->set_type(Debug_Msg_Debug_Type_LINE);
	msg->set_color(Debug_Msg_Color_RED);
	Debug_Line* line = msg->mutable_line();
	Point* p1 = line->mutable_start();
	Point* p2 = line->mutable_end();
	p1->set_x(st.getx());
	p1->set_y(st.gety());
	p2->set_x(end.getx());
	p2->set_y(end.gety());
	line->set_forward(forward);
	line->set_back(backward);
	return msg;
}

//int main(void)
//{
//	point end, start;
//	clock_t startTime, endTime;
//	std::vector<double> x, y;
//	Socket* fSock = new Socket(23333, 1);
//
//	end.setx(240);
//	end.sety(150);
//	start.setx(-240);
//	start.sety(-150);
//
//	RRTStar rrt;
//	while (1)
//	{
//		rrt.updateMap(fSock);
//		//startTime = clock();
//		rrt.rrt_star(start, end);
//		rrt.printPath(&x, &y);
//		getchar();
//		rrt.smooth(); //加了平滑可能会出现比较尖锐的拐角
//		//endTime = clock();
//		//cout << "Searching Time: " << endTime - startTime << endl;
//		rrt.printPath(&x, &y);
//		getchar();
//	}
//	
//	return 0;
//}