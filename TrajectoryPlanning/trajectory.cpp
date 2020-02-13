#include <iostream>
#include <math.h>
#include "trajectory.h"
using namespace std;
constexpr auto PI = 3.141593;
constexpr auto END_ANGLE = 0;
constexpr auto K1 = 1;
constexpr auto K2 = 5;
constexpr auto mu = 0.4;
constexpr auto lambda = 1;
constexpr auto RADIUS = 0.2; //路径点容差
constexpr auto RADIUS_END = 0.02; //终点容差
constexpr auto VMAX = 0.5;
constexpr auto WMAX = 0.5;
constexpr auto AW_MAX = 0.5;
constexpr auto AV_MAX = 1;
constexpr auto t = 0.016;
constexpr auto FPS = 60;


Trajectory::Trajectory(Socket* cSock, Socket* fSock)
{
	commandSock = cSock;
	frameSock = fSock;
}

void Trajectory::updateNodes(std::vector<double>* xpos, std::vector<double>* ypos)
{
	int i;
	ori.clear();
	x.assign(xpos->begin(), xpos->end());
	y.assign(ypos->begin(), ypos->end());
	for (i = 0; i < this->x.size() - 1; i++)
	{
		double dy = y[i + 1] - y[i];
		double dx = x[i + 1] - x[i];
		ori.push_back(atan2(dy, dx));
	}
	ori.push_back(ori[i-1]); //终点姿态
	cnt = 1; //cnt指向当前位置的目标点
}

void Trajectory::plan()
{
	int i = 0;
	double lastw = 0, lastv = 0;

	lastw = rotate();
	while (cnt < x.size())
	{
		if (cnt == x.size() - 1)
		{
			i = 1;
		}
		forward(i, &lastw, &lastv);
		//transit();
		cnt++;
		cout << "cnt = " << cnt << endl;
	}
	//停止
	commands.clear_command();
	Robot_Command* command = commands.add_command();
	command->set_robot_id(0);
	command->set_velocity_x(0);
	command->set_velocity_y(0);
	command->set_velocity_r(0);
	command->set_kick(true);
	command->set_power(0);
	command->set_dribbler_spin(0);
	commandSock->sendCommand(&commands);
	frameSock->receive(&frame);
	cout << "x: " << frame.robots_blue(0).x()/10 << endl;
	cout << "y: " << frame.robots_blue(0).y()/10 << endl;
	return;
}

void Trajectory::forward(int flag, double* lastw, double* lastv)
{
	clock_t time;
	double prevw = *lastw;
	double prevv = *lastv;

	while (1)
	{
		frameSock->receive(&frame);
		//计算距离
		double nowx = frame.robots_blue(0).x() / 10;
		double nowy = frame.robots_blue(0).y() / 10;
		double deltax = x[cnt] - nowx;
		double deltay = y[cnt] - nowy;
		//计算角度
		double rho = sqrt(pow(deltax, 2) + pow(deltay, 2)) / 100;
		//cout << "rho = " << rho << endl;
		double line_angle = atan2(deltay, deltax);
		double beta = -line_angle + ori[cnt];
		double theta = frame.robots_blue(0).orientation();
		double alpha = theta - line_angle;
		if (alpha > PI) //调整alpha范围在-pi~pi
			alpha -= 2 * PI;
		else if (alpha < -PI)
			alpha += 2 * PI;
		if (beta > PI) //调整beta范围在-pi~pi
			beta -= 2 * PI;
		else if (beta < -PI)
			beta += 2 * PI;
		//计算曲率
		double k = -(K2*(alpha - atan(-K1 * beta)) + sin(alpha)*(1 + K1 / (1 + pow(K1*beta, 2)))) / rho;
		//time = clock();
		//cout << time << endl;
		//cout << "k: "<< k << endl;

		//计算速度和角速度
		double v = VMAX / (1 + mu * pow(fabs(k), lambda)); //线速度本身不会超过限制
		//终点之前要减速
		double endDist = sqrt(pow(nowx - x.back(), 2) + pow(nowy - y.back(), 2)) / 100;
		//cout << "EndDist: " << endDist << endl;
		v = min(v, sqrt(2 * AV_MAX* (endDist-RADIUS_END)));
		//cout << "v_before: " << v << endl;
		//满足线加速度限制
		if (v > prevv + t * AV_MAX)
			v = prevv + t * AV_MAX;
		else if (v < prevv - t * AV_MAX)
			v = prevv - t * AV_MAX;
		//cout << "v_after: " << v << endl;
		double w = k * v;
		//cout << "w: " << w << endl;
		//cout << endl;
		//满足角速度限制
		if (w > WMAX)
			w = WMAX;
		else if(w < -WMAX)
			w = -WMAX;
		//满足角加速度限制
		if (w > prevw + t*AW_MAX)
			w = prevw + t*AW_MAX;
		else if (w < prevw - t*AW_MAX)
			w = prevw - t*AW_MAX;
		//cout << "w_after: " << w <<endl<<endl;

		prevw = w;
		prevv = v;

		//发送指令
		commands.clear_command();
		Robot_Command* command = commands.add_command();
		command->set_robot_id(0);
		command->set_velocity_x(100*v);
		command->set_velocity_y(0);
		command->set_velocity_r(-w);
		command->set_kick(true);
		command->set_power(0);
		command->set_dribbler_spin(0);
		commandSock->sendCommand(&commands);

		//检查是否到达target
		if (flag==0 && rho < RADIUS)
		{
			//把上一轮的值传回去，保证速度和角速度连续
			*lastv = v;
			*lastw = w;
			return;
		}
		else if (flag == 1 && rho < RADIUS_END)
		{
			int stopcnt = 1;
			//到达终点，角速度减为0
			while (fabs(w) > (double)AW_MAX/FPS)
			{
				w = (fabs(w) / w) * (fabs(w) - stopcnt * t*AW_MAX);
				cout << "end_w: " << w << endl;
				stopcnt++;
				commands.clear_command();
				Robot_Command* command = commands.add_command();
				command->set_robot_id(0);
				command->set_velocity_x(0);
				command->set_velocity_y(0);
				command->set_velocity_r(-w);
				command->set_kick(true);
				command->set_power(0);
				command->set_dribbler_spin(0);
				commandSock->sendCommand(&commands);
			}
			return;
		}

		/*if (rho < 20)
		{
			cout << "ax: " << frame.robots_blue(0).accelerate_x() << endl;
			cout << "w: " << frame.robots_blue(0).raw_rotate_vel() << endl;
		}*/
	}
	
}

double Trajectory::rotate()
{
	double w = 0;
	frameSock->receive(&frame);
	double theta = frame.robots_blue(0).orientation(); //实际角度
	double exp = ori[0]; //预期角度
	double delta = exp - theta; //角度差
	if (delta < 0)
	{
		delta += 2 * PI;
	}

	while (delta > 0.001)
	{
		w = min(WMAX, w + AW_MAX * t); //加速，匀速
		w = min(w, sqrt(2 * AW_MAX*(delta-0.001))); //减速
		//cout << "rotate w: " << w << endl;
		//发送指令
		commands.clear_command();
		Robot_Command* command = commands.add_command();
		command->set_robot_id(0);
		command->set_velocity_x(0);
		command->set_velocity_y(0);
		command->set_velocity_r(-w);
		command->set_kick(true);
		command->set_power(0);
		command->set_dribbler_spin(0);
		commandSock->sendCommand(&commands);

		frameSock->receive(&frame);
		theta = frame.robots_blue(0).orientation(); //实际角度
		delta = exp - theta; //角度差
		if (delta < 0)
		{
			delta += 2 * PI;
		}
	}
	return w;
}


//int main(void)
//{
//	std::vector<double> x, y;
//	Socket* fSock = new Socket(23333, 1);
//	Socket* cSock = new Socket(50001, 0);
//
//	x.push_back(-240);
//	x.push_back(0);
//	y.push_back(-150);
//	y.push_back(0);
//	
//	Trajectory trajectory(cSock, fSock);
//	trajectory.updateNodes(&x, &y);
//	trajectory.plan();
//	cout << endl;
//
//	return 0;
//}