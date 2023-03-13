#include "Command.h"

void Command::init()
{
	ReadUntilOK();
	initMap();		//这一步中将两个路线队列初始化完毕。
	Clear();
	Add_OK();
	Response();
}

void Command::start()
{
	while (ReadUntilOK()) {
		Clear();
		UpdateInfo();
		Add_frame();
		RobotDoWork();
		RobotSelectWork();
		Add_OK();
		Response();
	}
}

bool Command::ReadUntilOK()
{
}

void Command::Clear()
{
}

void Command::Add_OK()
{
}

void Command::Add_frame()
{
}

void Command::Response()
{
}

void Command::initMap()
{
	//循环地图数据，先保存下所有工作台和机器人的初始位置，此处默认缓冲区buf末尾没有ok。
	for (int i = 0; i < buf.size(); ++i) {
		for (int j = 0; j < buf[0].size(); ++j) {
			if (buf[i][j] == 'A') {

			}
		}
	}
}

void Command::UpdateInfo()
{
}

void Command::RobotDoWork()
{
}

void Command::RobotSelectWork()
{
}

void Command::mapToreal(pair<int, int> old, pair<double, double>& ret)
{
	ret.first = static_cast<double>(old.first) / 2 + 0.25;
	ret.second = static_cast<double>(old.second) / 2 + 0.25;
}

void Command::realTomap(pair<double, double> old, pair<int, int>& ret)
{
	ret.first = (old.first - 0.25) * 2;
	ret.second = (old.second - 0.25) * 2;
}
