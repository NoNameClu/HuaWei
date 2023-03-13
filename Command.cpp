#include "Command.h"

const unordered_set<int> Command::style{ 1,2,3,4,5,6,7,8,9 };

void Command::init()
{
	ReadUntilOK();
	initMap();		//��һ���н�����·�߶��г�ʼ����ϡ�
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
	//ѭ����ͼ���ݣ��ȱ��������й���̨�ͻ����˵ĳ�ʼλ�ã��˴�Ĭ�ϻ�����bufĩβû��ok��
	for (int i = 0; i < buf.size(); ++i) {
		for (int j = 0; j < buf[0].size(); ++j) {
			if (buf[i][j] == 'A') {
				robot temp;
				temp.state = NONE;
				robots.push_back(temp);
			}
			if (style.find(buf[i][j] - '0') != style.end()) {
				worker temp;
				pair<int, int> pos = make_pair(i, j);
				pair<double, double> real_pos;
				mapToreal(pos, real_pos);
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[i][j] - '0';
				switch (temp.style) {
				case 1:
					temp.need_money = 3000;
					temp.sell_money = 6000;
					break;
				case 2:
					temp.need_money = 4400;
					temp.sell_money = 7600;
					break;
				case 3:
					temp.need_money = 5800;
					temp.sell_money = 9200;
					break;
				case 4:
					temp.need_money = 15400;
					temp.sell_money = 22500;
					temp.need_object |= (OBJECT_ONE | OBJECT_TWO);
					break;
				case 5:
					temp.need_money = 17200;
					temp.sell_money = 25000;
					temp.need_object |= (OBJECT_ONE | OBJECT_THREE);
					break;
				case 6:
					temp.need_money = 19200;
					temp.sell_money = 27500;
					temp.need_object |= (OBJECT_TWO | OBJECT_THREE);
					break;
				case 7:
					temp.need_money = 76000;
					temp.sell_money = 105000;
					temp.need_object |= (OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX);
					break;
				case 8:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object |= (OBJECT_SEVEN);
					break;
				case 9:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object |= (OBJECT_ONE | OBJECT_TWO | OBJECT_THREE | OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX | OBJECT_SEVEN);
				default:
					break;
				}
				idToworker[i * 100 + j] = temp;
			}
		}
	}

	//ѭ���жϼ���·��
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
