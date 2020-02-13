#include <iostream>
#include <vector>
#include <algorithm>
#include "path.h"
#include "trajectory.h"
#include "socket.h"
using namespace std;

int main(void)
{
	std::vector<double> x, y;
	point end, start, tmp;
	clock_t startTime, endTime;
	int i = 0;
	Socket* fSock = new Socket(23333, 1);
	Socket* cSock = new Socket(50001, 0);
	Vision_DetectionFrame frame;

	end.setx(240);
	end.sety(150);
	start.setx(-240);
	start.sety(-150);

	RRTStar rrt;
	Trajectory trajectory(cSock, fSock);

	for (; i < 10; i++)
	{
		fSock->receive(&frame);	//����udp��Ϣ
		rrt.updateMap(frame);
		startTime = clock();
		rrt.rrt_star(start, end);
		rrt.smooth(); //����ƽ�����ܻ���ֱȽϼ���Ĺս�
		endTime = clock();
		cout << "Searching Time: " << endTime - startTime << endl;
		rrt.printPath(&x, &y);
		cout << "Running" << endl;
		trajectory.updateNodes(&x, &y);
		trajectory.plan();
		cout << endl;
		std::reverse(x.begin(), x.end());
		std::reverse(y.begin(), y.end());

		//�������յ�
		tmp = start;
		start = end;
		end = tmp;
	}

	return 0;
}