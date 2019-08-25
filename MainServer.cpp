


#include "Aria.h"
#include <iostream>
#include <stdio.h>
#include <conio.h> //window 平台
#include <WinSock2.h>
#include <list>
#pragma comment(lib, "ws2_32")
using namespace std;
#define random(x) (rand()%x)

class ActionGo : public ArAction
{
public:
	// constructor, sets myMaxSpeed and myStopDistance
	ActionGo(double maxSpeed, double stopDistance);
	// destructor. does not need to do anything
	virtual ~ActionGo(void) {};
	// called by the action resolver to obtain this action's requested behavior
	virtual ArActionDesired *fire(ArActionDesired currentDesired);
	// store the robot pointer, and it's ArSonarDevice object, or deactivate this action if there is no sonar.
	virtual void setRobot(ArRobot *robot);
protected:
	// the sonar device object obtained from the robot by setRobot()
	ArRangeDevice *mySonar;


	
	ArActionDesired myDesired;

	double myMaxSpeed;
	double myStopDistance;
};


/* Action that turns the robot away from obstacles detected by the
* sonar. */
class ActionTurn : public ArAction
{
public:
	// constructor, sets the turnThreshold, and turnAmount
	ActionTurn(double turnThreshold, double turnAmount);
	// destructor, its just empty, we don't need to do anything
	virtual ~ActionTurn(void) {};
	// fire, this is what the resolver calls to figure out what this action wants
	virtual ArActionDesired *fire(ArActionDesired currentDesired);
	// sets the robot pointer, also gets the sonar device, or deactivates this action if there is no sonar.
	virtual void setRobot(ArRobot *robot);
protected:
	// this is to hold the sonar device form the robot
	ArRangeDevice *mySonar;
	// what the action wants to do; used by the action resover after fire()
	ArActionDesired myDesired;
	// distance at which to start turning
	double myTurnThreshold;
	// amount to turn when turning is needed
	double myTurnAmount;
	// remember which turn direction we requested, to help keep turns smooth
	int myTurning; // -1 == left, 1 == right, 0 == none
};


ActionGo::ActionGo(double maxSpeed, double stopDistance) :
	ArAction("Go")
{
	mySonar = NULL;
	myMaxSpeed = maxSpeed;
	myStopDistance = stopDistance;
	setNextArgument(ArArg("maximum speed", &myMaxSpeed, "Maximum speed to go."));
	setNextArgument(ArArg("stop distance", &myStopDistance, "Distance at which to stop."));
}


void ActionGo::setRobot(ArRobot *robot)
{
	ArAction::setRobot(robot);
	mySonar = robot->findRangeDevice("sonar");
	if (robot == NULL)
	{
		ArLog::log(ArLog::Terse, "actionExample: ActionGo: Warning: I found no sonar, deactivating.");
		deactivate();
	}
}


ArActionDesired *ActionGo::fire(ArActionDesired currentDesired)
{
	double range;
	double speed;

	// reset the actionDesired (must be done), to clear
	// its previous values.
	myDesired.reset();

	// if the sonar is null we can't do anything, so deactivate
	if (mySonar == NULL)
	{
		deactivate();
		return NULL;
	}
	// get the range of the sonar
	range = mySonar->currentReadingPolar(-70, 70) - myRobot->getRobotRadius();
	// if the range is greater than the stop distance, find some speed to go
	if (range > myStopDistance)
	{
		// just an arbitrary speed based on the range
		speed = range * .3;
		// if that speed is greater than our max, cap it
		if (speed > myMaxSpeed)
			speed = myMaxSpeed;
		// now set the velocity
		myDesired.setVel(speed);
	}
	// the range was less than the stop distance, so request stop
	else
	{
		myDesired.setVel(0);
	}
	// return a pointer to the actionDesired to the resolver to make our request
	return &myDesired;
}



ActionTurn::ActionTurn(double turnThreshold, double turnAmount) :
	ArAction("Turn")
{
	myTurnThreshold = turnThreshold;
	myTurnAmount = turnAmount;
	setNextArgument(ArArg("turn threshold (mm)", &myTurnThreshold, "The number of mm away from obstacle to begin turnning."));
	setNextArgument(ArArg("turn amount (deg)", &myTurnAmount, "The number of degress to turn if turning."));
	myTurning = 0;
}


void ActionTurn::setRobot(ArRobot *robot)
{
	ArAction::setRobot(robot);
	mySonar = robot->findRangeDevice("sonar");
	if (mySonar == NULL)
	{
		ArLog::log(ArLog::Terse, "actionExample: ActionTurn: Warning: I found no sonar, deactivating.");
		deactivate();
	}
}

/*
This is the guts of the Turn action.
*/
ArActionDesired *ActionTurn::fire(ArActionDesired currentDesired)
{
	double leftRange, rightRange;
	// reset the actionDesired (must be done)
	myDesired.reset();
	// if the sonar is null we can't do anything, so deactivate
	if (mySonar == NULL)
	{
		deactivate();
		return NULL;
	}
	// Get the left readings and right readings off of the sonar
	leftRange = (mySonar->currentReadingPolar(0, 100) -
		myRobot->getRobotRadius());
	rightRange = (mySonar->currentReadingPolar(-100, 0) -
		myRobot->getRobotRadius());
	// if neither left nor right range is within the turn threshold,
	// reset the turning variable and don't turn
	if (leftRange > myTurnThreshold && rightRange > myTurnThreshold)
	{
		myTurning = 0;
		myDesired.setDeltaHeading(0);
	}
	// if we're already turning some direction, keep turning that direction
	else if (myTurning)
	{
		myDesired.setDeltaHeading(myTurnAmount * myTurning);
	}
	
	else if (leftRange < rightRange)
	{
		myTurning = -1;
		myDesired.setDeltaHeading(myTurnAmount * myTurning);
	}
	
	else
	{
		myTurning = 1;
		myDesired.setDeltaHeading(myTurnAmount * myTurning);
	}
	// return a pointer to the actionDesired, so resolver knows what to do
	return &myDesired;
}



int main(int argc, char** argv)
{

	ArRobot robot;
	//ArGripper gripper(&robot);
	Aria::init(); //初始化
	ArSimpleConnector connector(&argc, argv);
	ArArgumentParser parser(&argc, argv);
	ArSonarDevice sonar;

	parser.loadDefaultArguments();
	if (!connector.parseArgs() || argc>1)
	{
		connector.logOptions();
		exit(1);
	}
	if (!connector.connectRobot(&robot))
	{
		printf("cound not connect to robot...exiting\n");
		Aria::shutdown();
		return 1;
	}


	//加载套接字库
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	wVersionRequested = MAKEWORD(1, 1);//准备加载的Winsock库的版本 1.1

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		return -1;   //加载不成功则退出
	}


	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion) != 1) {
		WSACleanup();
		return -1;
	}


	SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);//创建套接字 第一个参数Address families
														//AF_INET, internetwork: UDP, TCP, etc.
														//SOCK_DGRAM数据报套接字
														//第三个参数为0，自动选择协议


	SOCKADDR_IN addrSrv;
	addrSrv.sin_addr.S_un.S_addr = inet_addr("172.20.10.9");//IP地址
	addrSrv.sin_family = AF_INET; //地址族
	addrSrv.sin_port = htons(8888);//分配给套接字的端口

	SOCKADDR_IN addrClient; //定义一个用于存储客户端IP地址和端口信息的缓冲区
	int len = sizeof(SOCKADDR);
	
	connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));

	send(sockClient, "machine", strlen("machine") + 1, 0);
	send(sockClient, "To:phone@content", strlen("To:phone@content") + 1, 0);
	robot.comInt(ArCommands::ENABLE, 1);
	robot.comInt(ArCommands::SOUNDTOG, 0);
	ActionGo go(500, 350);
	ActionTurn turn(400, 10);
	ArActionStallRecover recover;
	robot.runAsync(true);

	//int son = 0;
	//}
	while (1) {
		char recvBuf[100] = "";
		//memset(recvBuf, 0, 100 * sizeof(char));
		recv(sockClient, recvBuf, 10, 0);
		//printf("%s\n", recvBuf);
			cout << robot.getX() << "  " << robot.getY() << "  " << robot.getTh() << " " << robot.getBatteryVoltage() << endl;
			//robot.getSonarRange(4);
			
			switch (recvBuf[0])
			{
			case 'W':
				
				robot.setVel(350);
				cout << robot.getVel() << endl;
				while (1) { if (robot.isRunning()) { break; } }
				break;
			case 'E':

				robot.setVel(450);
				cout << robot.getVel() << endl;
				while (1) { if (robot.isRunning()) { break; } }
				break;

			case 'R':
				robot.setVel(650);
				cout << robot.getVel() << endl;
				while (1) { if (robot.isHeadingDone()) { break; } }
				break;
			case 'Z':
				robot.setDeltaHeading(45);

				while (1) { if (robot.isHeadingDone()) { break; } }
				break;

			case 'X':
				robot.setDeltaHeading(-45);

				while (1) { if (robot.isHeadingDone()) { break; } }
				break;

			case 'C':
				robot.setDeltaHeading(-135);

				while (1) { if (robot.isHeadingDone()) { break; } }
				break;

			case 'V':
				robot.setDeltaHeading(135);

				while (1) { if (robot.isHeadingDone()) { break; } }
				break;

			case 'S':
				robot.setVel(-450);
				cout << robot.getVel() << endl;
				while (1) { if (robot.isHeadingDone()) { break; } }
				break;

			case 'A':
				robot.setDeltaHeading(90);

				while (1) { if (robot.isHeadingDone()) { break; } }

				break;

			case 'D':
				robot.setDeltaHeading(-90);

				while (1) { if (robot.isHeadingDone()) { break; } }
				break;
			case 'Q':
				robot.stop();
				//while (1) { if (robot.isRunning()) { break; } }
				break;
			case 'J':
				cout << "开启矩形走模式" << endl;
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(90);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 90 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(180);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 180 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(270);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 270 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				break;
			case 'K':
				cout << "开启梯形走模式" << endl;
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setDeltaHeading(120);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 120 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setDeltaHeading(120);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 240 deg\n");
				robot.move(1500);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				break;
			case 'L':

				cout << "开启蛇形走模式" << endl;

				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(90);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 90 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(-90);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at -90 deg\n");
				robot.move(1000);
				while (1) { if (robot.isMoveDone()) { break; } }
				robot.setHeading(0);
				while (1) { if (robot.isHeadingDone()) { break; } }
				printf("at 0 deg\n");

				break;

			case'T':
				
			
				robot.addRangeDevice(&sonar);
				robot.addAction(&recover, 100);
				robot.addAction(&go, 50);
				robot.addAction(&turn, 49);

				break;
		

			}
			


	}

}
/*	case 'S':
robot.setVel2(-moveLevel, -moveLevel);
break;

case 'D':
robot.setVel2(100, -100);
break;

case 'Q':
if (moveLevel > 25)
moveLevel -= 25;
robot.setVel2(moveLevel, moveLevel);
break;

case 'W':
robot.setVel2(moveLevel, moveLevel);
break;
case 'E':
if (moveLevel < 400)
moveLevel += 25;
robot.setVel(moveLevel);
break;
case ' ':
robot.stop();
break;
case 'U':
gripper.liftUp();
break;
case 'J':
gripper.liftDown();
break;
case 'O':
gripper.gripOpen();
break;
case 'P':
gripper.gripClose();
break;
case 'K':
gripper.gripperHalt();
break;
}
}
//若recv接收到，并且longth为0，说明客户端退出，终止线程
/*else
{
cout << inet_ntoa(addrClient.sin_addr) << "\thave quit!" << endl;

gripper.gripperHalt();
//bConnected = 0;
robot.stop();
return 0;
}*/

//}

/* DWORD WINAPI PositionMessageThread(LPVOID lparam)
{
mysockInfo = *(SockInfo*)lparam;
SOCKET sockClient = mysockInfo.sockClient;
SOCKADDR_IN addrClient = mysockInfo.addrClient;
char positionStr[64];
while (1)
{
if (bConnected = 1)
{
sprintf(positionStr, "%lf\t%lf\0", robot.getX(), robot.getY());
send(sockClient, positionStr, 64, 0);
}
else
return 0;

Sleep(50);

}

}

*/

