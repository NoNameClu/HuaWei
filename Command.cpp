#include "Command.h"

#define DEBUG

const unordered_set<int> Command::style{ 1,2,3,4,5,6,7,8,9 };

void Command::init()
{
	ReadUntilOK();
	initMap();		//��һ���н�����·�߶��г�ʼ����ϡ�
	Add_OK();
	Response();
	Clear();
}

void Command::start()
{
	while (ReadUntilOK()) {
		UpdateInfo();
		Add_frame();
		RobotDoWork();
		RobotSelectWork();
		Add_OK();
		Response();
		Clear();
	}
}

bool Command::ReadUntilOK()
{
#ifdef DEBUG
	cerr << "��ʼ��֡" << endl;
#endif // DEBUG

	string line;
	while (getline(cin, line)) {
		if (line[0] == 'O' && line[1] == 'K')
			return true;

		buf.push_back(line);

#ifdef DEBUG
		//cerr << line << endl;
#endif // DEBUG

	}
	return false;
}

void Command::Clear()
{
	buf.clear();
	response.clear();
}

void Command::Add_OK()
{
	response.push_back("OK");
}

void Command::Add_frame()
{
	response.push_back(to_string(frame));
}

void Command::Response()
{
	for (const auto& res : response) {
		cout << res << endl;
#ifdef DEBUG
		cerr << res << endl;
#endif // DEBUG

	}
}

void Command::initMap()
{
#ifdef DEBUG
	cerr << "����initmap����" << endl;
#endif

	//ѭ����ͼ���ݣ��ȱ��������й���̨�ͻ����˵ĳ�ʼλ�ã��˴�Ĭ�ϻ�����bufĩβû��ok��
	for (int i = buf.size()-1; i >=0; i--) {	//	�����һ�����Ͻ�����ͼ
		for (int j = 0; j < 100; ++j) {
			if (buf[i][j] == 'A') {		// ������
				robot temp;
				temp.state = NONE;
				robots.push_back(temp);
			}
			if (style.find(buf[i][j] - '0') != style.end()) {	// ����̨
				worker temp;
				pair<int, int> pos = make_pair(buf.size()-1-i, j);	//	����̨int����
				pair<double, double> real_pos;
				mapToreal(pos, real_pos);				//	int����תdoubleʵ������
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[i][j] - '0';
				switch (temp.style) {
				case 1:
					temp.need_money = 3000;
					temp.sell_money = 6000;
					temp.need_object = OBJECT_NULL;
					temp.product_object = OBJECT_ONE;
					break;
				case 2:
					temp.need_money = 4400;
					temp.sell_money = 7600;
					temp.need_object = OBJECT_NULL;
					temp.product_object = OBJECT_TWO;
					break;
				case 3:
					temp.need_money = 5800;
					temp.sell_money = 9200; 
					temp.need_object = OBJECT_NULL;
					temp.product_object = OBJECT_THREE;
					break;
				case 4:
					temp.need_money = 15400;
					temp.sell_money = 22500;
					temp.product_object = OBJECT_FOUR;
					temp.need_object = (OBJECT_ONE | OBJECT_TWO);
					break;
				case 5:
					temp.need_money = 17200;
					temp.sell_money = 25000;
					temp.product_object = OBJECT_FIVE;
					temp.need_object = (OBJECT_ONE | OBJECT_THREE);
					break;
				case 6:
					temp.need_money = 19200;
					temp.sell_money = 27500;
					temp.product_object = OBJECT_SIX;
					temp.need_object = (OBJECT_TWO | OBJECT_THREE);
					break;
				case 7:
					temp.need_money = 76000;
					temp.sell_money = 105000;
					temp.product_object = OBJECT_SEVEN;
					temp.need_object = (OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX);
					break;
				case 8:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object = (OBJECT_SEVEN);
					break;
				case 9:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object = (OBJECT_ONE | OBJECT_TWO | OBJECT_THREE | OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX | OBJECT_SEVEN);
				default:
					break;
				}
				idToworker[i * 100 + j] = temp;
			}
		}
	}

	//ѭ���жϼ���·��
	for (auto& p : idToworker) {
		int start_id = p.first;
		worker start = p.second;
		for (auto& q : idToworker) {
			int end_id = q.first;
			worker end = q.second;
			if (start_id == end_id) {
				continue;
			}
			if ((start.product_object & end.need_object) == 0) {	//	ǰ���������Ǻ�����Ҫ�ģ���·�߲���Ч
				continue;
			}
			route temp(start_id, end_id);
			temp.base = start.need_money;
			temp.value = start.sell_money - start.need_money;
			if (end.style >= 4 && end.style <= 6) {
				temp.value += (end.sell_money - end.need_money) / 2;
			}
			else if(end.style ) {
				temp.value += (end.sell_money - end.need_money) / 3;
			}
			temp.object = start.product_object;
			temp.length = GetLength(start.real_pos, end.real_pos);
			temp.stat = NO_PRODUCT ;
			unavaliable.push_back(temp);
		}
	}

#ifdef DEBUG
	cerr << "�ܹ���" << unavaliable.size() << "·��" << endl;
#endif // DEBUG


}

void Command::UpdateInfo()
{
#ifdef DEBUG
	cerr << "����UpdateInfo����" << endl;
#endif // DEBUG

	string a;

	//	����һ��
	int index = 0;
	a = buf[index++];
	stringstream sa;
	sa.str(a);
	sa >> frame;

#ifdef DEBUG
	cerr << frame << endl;
#endif // DEBUG

	int m_temp;
	sa >> m_temp;

#ifdef DEBUG
	cerr << m_temp << endl;
#endif // DEBUG


	if (m_temp != money) {
		stat = WRONG;
		Clean_list();
	}

#ifdef DEBUG
	cerr << (stat == WRONG ? "wrong" : "normal") << endl;
#endif // DEBUG


	money = m_temp;
	flush_money_stat();

	//	���ڶ���
	a = buf[index++];

#ifdef DEBUG
	//cerr << buf[index - 1] << endl;
#endif // DEBUG

	stringstream sb(a);
	sb >> worker_num;

#ifdef DEBUG
	cerr << "���¹���̨ǰ" << endl;
	//cerr << buf.size() << endl;
	//cerr << worker_num << endl;
#endif // DEBUG


	int n = worker_num;

#ifdef DEBUG
	//cerr << n << endl;
#endif // DEBUG


	while (n--) {
		//	����3��n+3��
		a = buf[index++];
		stringstream s_for;
		s_for.str(a);

		worker tmp;
		s_for >> tmp.style;
		pair<double, double> real_pos;
		s_for >> real_pos.first;
		s_for >> real_pos.second;
		tmp.real_pos = real_pos;	//	����ת��
		realTomap(tmp.real_pos, tmp.pos); 

		int time;
		s_for >> time;		// ����̨ʣ�������ʱ�䣨û���õ���
		tmp.time = time;
		s_for >> tmp.hold_object;
		s_for >> tmp.output;

#ifdef DEBUG
		//cerr << tmp.real_pos.first << " " << tmp.real_pos.second << " " << tmp.hold_object << " " << tmp.output << endl;
#endif // DEBUG


		int id = tmp.pos.first * 100 + tmp.pos.second;
		// tmp2��¼�˴˹���̨��һ֡��״̬������û���뵱ǰ֡�Ƚ��д�
		if (tmp.output) {

#ifdef DEBUG
			//cerr << "�ѵ�product������" << endl;
#endif // DEBUG


			takeoff_product_stat(id);
		}
		idToworker[id] = tmp;
	}

	bool flag = true;

#ifdef DEBUG
	cerr << "���»�����֮ǰ" << endl;
#endif // DEBUG


	for (int i = 0; i < 4; i++) {
		a = buf[index++];
		stringstream s_for;
		s_for.str(a);

		int workerID;
		s_for >> workerID;
		int item;
		s_for >> item;

		double time_weight, collide_weight;		// ʱ���ֵϵ������ײ��ֵϵ����û������
		s_for >> time_weight;
		s_for >> collide_weight;

		double a_speed,face;
		pair<double, double> l_speed, real_pos;
		s_for >> a_speed >> l_speed.first >> l_speed.second;
		s_for >> face;
		s_for >> real_pos.first >> real_pos.second;
		
		//robots[i].on_job = item > 0;	// Я����Ʒ��Ŵ���0�ͱ�ʾ�ڹ�����
		robots[i].face = face;			// ���»�������Ϣ
		robots[i].a_speed = a_speed;
		robots[i].l_speed = l_speed;
		robots[i].real_pos = real_pos;
		if (robots[i].on_job) {
			if (robots[i].state == BEFORE && isNear(robots[i].real_pos, idToworker[robots[i].cur.start].real_pos, 0.4)) {
				robots[i].can_buy = true;
			}
			if (robots[i].state == AFTER && isNear(robots[i].real_pos, idToworker[robots[i].cur.end].real_pos, 0.4)) {
				robots[i].can_sell = true;
			}
		}
		if (robots[i].l_speed.first != 0 || robots[i].l_speed.second != 0 || robots[i].a_speed != 0) {
			flag = false;
		}
	}

	if (flag && stat == WRONG) {
		stat = NORMAL;
	}

	takeoff_need_stat();
	flush_list();
}


//	��ײ���ģ��
void Command::collision_avoidance()
{
	for (int i = 0; i < robots.size()-1; i++) {
		robot rb1 = robots[i];
		for (int j = i+1; j < robots.size(); j++) {
			robot rb2 = robots[j];
			if (will_collision(rb1, rb2) == 1) {
				//	��ת�ǣ���rb1,rb2 ��ת+30��

			}
			else if (will_collision(rb1, rb2) == 2) {
				//	rb2 ����
			}
			else if (will_collision(rb1, rb2) == 3) {
				//	rb1����
			}
		}
	}
}

int Command::will_collision(robot rb1, robot rb2)
{
	pair<double, double> rb1_pos = rb1.real_pos;
	pair<double, double> rb2_pos = rb2.real_pos;
	double distance1 = GetLength(rb1_pos, rb2_pos);	//	����������֮�����ľ�
	double distance2 = (abs(rb2_pos.first * tan(rb1.face) - rb2_pos.second +
		rb1_pos.second - rb1_pos.first * tan(rb1.face))) / sqrt(pow(tan(rb1.face), 2) + 1);
	double dis_face = abs(rb1.face - rb2.face);
	if (isNear(rb1.real_pos, idToworker[rb1.cur.start].real_pos, 0.4) && dis_face < 1e-3 && distance2 < 0.6 && distance1 < 1.6) return 2;
	if (isNear(rb2.real_pos, idToworker[rb2.cur.start].real_pos, 0.4) && dis_face < 1e-3 && distance2 < 0.6 && distance1 < 1.6) return 3;
	if (dis_face < 1e-3 && distance2 < 0.6 && distance1 < 1.6) return 1;
	return 0;
}

void Command::RobotDoWork()
{
#ifdef DEBUG
	cerr << "����dowork����" << endl;
#endif // DEBUG

	if (stat == WRONG) {	//	״̬����ʱ����ջ�������Ʒ
		for (int i = 0; i < robots.size(); ++i) {
			string first = "", second = "", third = "";
			first += "forward ";
			first += to_string(i);
			first += " ";
			first += to_string(0);
			second += "rotate ";
			second += to_string(i);
			second += " ";
			second += to_string(0);
			third += "destroy ";
			third += to_string(i);
			response.push_back(first);
			response.push_back(second);
			response.push_back(third);
			robots[i].can_buy = false;
			robots[i].can_sell = false;
		}
		return;
	}

	//ѭ���ж���Щrobot�Ѿ���ѡ����·��
	for (int i = 0; i < robots.size(); ++i) {
		worker next;
		string fd, ro;
		robot rt = robots[i];
#ifdef DEBUG
		//cerr << "������" << i << endl;
#endif // DEBUG

		if (!rt.on_job) {
			continue;
		}
#ifdef DEBUG
		//cerr << "������" << i << "��ʼ����" << endl;
#endif // DEBUG

		if (rt.can_buy && rt.state == BEFORE) {	//	�ܹ��򣨿���Ŀ�깤��̨��
			string temp = "";
			temp += "buy ";
			temp += to_string(i);
			response.push_back(temp);

			robots[i].state = AFTER;
			robots[i].can_buy = false;
			money -= robots[i].cur.base;

			puton_product_stat(rt.cur.start);
		}

		if (rt.can_sell && rt.state == AFTER) {	//	�ܳ���
			string temp = "";
			temp += "sell ";
			temp += to_string(i);
			response.push_back(temp);

			robots[i].state = NONE;
			robots[i].can_sell = false;
			robots[i].on_job = false;
			money += robots[i].cur.value + robots[i].cur.base;
			
			puton_need_stat(rt.cur.end, rt.cur.object);
		}

		switch (rt.state) {
		case BEFORE:
			next = idToworker[rt.cur.start];
			break;
		case AFTER:
			next = idToworker[rt.cur.end];
			break;
		case NONE:
			fd += "forward ";
			fd += to_string(i);
			fd += " ";
			fd += to_string(0);
			ro += "rotate ";
			ro += to_string(i);
			ro += " ";
			ro += to_string(0);
			response.push_back(fd);
			response.push_back(ro);
			return;
		}

#ifdef DEBUG
		//cerr << "������" << i << "��ʼ����" << endl;
#endif // DEBUG


		//�ҵ���Ŀ��
		double speed, angle;
		double x = next.real_pos.second - rt.real_pos.second,
			y = (next.real_pos.first - rt.real_pos.first);
		double target_angle = atan2(x, y);	//	����Ŀ��Ƕ���x������н�
		double a_diff = target_angle + (-rt.face);	//	����Ŀ��Ƕ��볯��Ĳ�ֵ
		if (a_diff > M_PI) {
			a_diff -= 2 * M_PI;
		}
		else if (a_diff < (-M_PI)) {
			a_diff += 2 * M_PI;
		}
		int dir = a_diff > 0 ? 1 : -1;
		a_diff = abs(a_diff);

#ifdef DEBUG
		cerr << GetLength(rt.real_pos, next.real_pos) << "������" << i << endl;
		cerr << "Ŀ�ĵ���" << next.real_pos.first << " " << next.real_pos.second << "��������" << rt.real_pos.first << " " << rt.real_pos.second << endl;
		cerr << "ƫ����" << a_diff * (180 / M_PI) << endl;;
#endif // DEBUG


		//�������˺�Ŀ��ص�ƫ�Ǻܴ�
		if (a_diff >= M_PI_2) {
			speed = 0.1;
			angle = M_PI * dir;
		}
		//�������˺�Ŀ��ص�ƫ�ǽϴ�
		else if(a_diff >= M_PI_8) {
			speed = 2;
			angle = M_PI_2 * dir;
		}
		//��΢��  
		else if(a_diff >= M_PI_32) {
			speed = 3;
			angle = M_PI_8 * dir;
		}
		else if (a_diff >= M_PI_64) {
			speed = 5;
			angle = M_PI_16 * dir;
		}
		//ǰ��
		else {
			speed = 6;
			angle = 0;
		}

		fd += "forward ";
		fd += to_string(i);
		fd += " ";
		fd += to_string(speed);
		ro += "rotate ";
		ro += to_string(i);
		ro += " ";
		ro += to_string(angle);
		response.push_back(fd);
		response.push_back(ro);
	}

flush_money_stat();
flush_list();
}

void Command::RobotSelectWork()
{
#ifdef DEBUG
	cerr << "����select����" << endl;
	cerr << "��" << avaliable.size() << "��·��" << endl;
#endif // DEBUG

	if (stat == WRONG) {
		for (int i = 0; i < robots.size(); ++i) {
			robots[i].on_job = false;
			return;
		}
	}

	for (int i = 0; i < robots.size(); i++) {
		//�жϵ�ǰ�������Ƿ��й���
		//�õ���ǰ�����˵����꣬
		//������ѡ��·�ߣ�ͳ�ƿ�ѡ·�߸����ĳ���
		//first���ܳ��ȣ�second��·��
		robot rb = robots[i];
		if (rb.on_job) continue;	// ��ǰ������û�з��乤��


		//	Ѱ��maxValue��maxDistance�������һ���Լ�����
		double maxValue = 0, maxDistance = 0;
		for (auto& cur_route : avaliable) {
			worker route_start_worker = idToworker[cur_route.start];
			pair<double, double> cur_worker_pos = route_start_worker.real_pos;
			double distance = GetLength(rb.real_pos, cur_worker_pos);	//	��������˵���ʼ�����
			distance += cur_route.length;
			maxValue = max(maxValue, cur_route.value);
			maxDistance = max(maxDistance, distance);
		}
		//	����һ��������֮���ָ���Ȩ���ָ�·�ߣ�Ȼ��Ž�priority_queue

		auto cmp = [](const pair<double, route>& lhs, const pair<double, route>& rhs) {
			return lhs.first < rhs.first;
		};

		priority_queue<pair<double, route>, vector<pair<double, route>>, decltype(cmp)> pq(cmp);	//	�������double����
		for (auto cur_route : avaliable) {
			worker route_start_worker = idToworker[cur_route.start];
			pair<double, double> cur_worker_pos = route_start_worker.real_pos;
			double distance = GetLength(rb.real_pos, cur_worker_pos);	//	��������˵���ʼ�����
			distance += cur_route.length;
			distance = distance / maxDistance;	//·��ԽСԽ�ã�����һ����С��ָ��
			maxValue = 1 - cur_route.value / maxValue;	// ��ֵԽ��Խ�ã���һ��������ָ�꣬��Ҫ����
			double score = distance * 0.5 + maxValue * 0.5;	//	Ȩ�ض���0.5
			pq.push(make_pair(score, cur_route));	//ѡ�е�·�߸������֣�ԽСԽ�ã�����	
		}

		if (pq.empty()) {
			return;
		}

		pair<double, route> my_route = pq.top();
		pq.pop();

		//��ȡ���������·��
		//start,end
		//ˢ��·��
		puton_occ_stat(my_route.second.start);
		puton_occ_stat(my_route.second.end, my_route.second.object);
		flush_list();
		robots[i].cur = my_route.second;
		robots[i].on_job = true;
		robots[i].state = BEFORE;

#ifdef DEBUG
		cerr << "robot��ʼ����" << endl;
#endif // DEBUG


	}
}

void Command::mapToreal(pair<int, int> old, pair<double, double>& ret)
{
	ret.first = static_cast<double>(old.first) / 2 + 0.25;
	ret.second = 50 - (static_cast<double>(old.second) / 2 + 0.25);
}

void Command::realTomap(pair<double, double> old, pair<int, int>& ret)
{
	ret.first = (old.first - 0.25) * 2;
	ret.second = 99 - (old.second - 0.25) * 2;
}

//	��������֮��ľ���
double Command::GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs) {
	double x = lhs.first - rhs.first, y = lhs.second - rhs.second;
	return sqrt(x * x + y * y);
}

bool Command::isNear(const pair<double, double>& lhs, const pair<double, double>& rhs, double dis)
{
	if (GetLength(lhs, rhs) <= dis) {
		return true;
	}
	return false;
}

//	���avaliable������unavaliable
void Command::Clean_list() {
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		unavaliable.insert(unavaliable.end(), *p);
		p = avaliable.erase(p);
	}
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		p->stat = NO_PRODUCT;
		if ((idToworker[p->end].hold_object & p->object) != 0){
			p->stat |= NO_NEED;
		}
	}
}

//	���±�ѡ�е�·��
void Command::flush_list() {
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else {
			++p;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end();) {
		if (p->stat == 0) {
			avaliable.insert(avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
		else {
			++p;
		}
#ifdef DEBUG
		//cerr << p->stat << endl;
#endif // DEBUG
	}

#ifdef DEBUG
	cerr << "·��" << avaliable.size() << " " << unavaliable.size() << endl;
	/*if (avaliable.size() == 0) {
		for (const auto& p : unavaliable) {
			cerr << "ԭ��" << p.stat << endl;
		}
	}*/
#endif // DEBUG

}

void Command::flush_money_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->base <= money) {
			p->stat &= (~NO_MONEY);
		}
		if (p->base > money) {
			p->stat |= NO_MONEY;
		}
	}
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->base > money) {
			p->stat |= NO_MONEY;
		}
	}
}

void Command::takeoff_product_stat(int id) {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~NO_PRODUCT);
		}
	}
}

void Command::puton_product_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC);
			p->stat |= NO_PRODUCT;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC);
			p->stat |= NO_PRODUCT;
		}
	}
}

//start�汾
void Command::puton_occ_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC;
		}
	}
}

//end�汾
void Command::puton_occ_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC;
		}
	}
}

void Command::puton_need_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC);
			p->stat |= NO_NEED;
		}
	}
}

void Command::takeoff_need_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if ((idToworker[p->end].hold_object & p->object) == 0) {
			p->stat &= (~NO_NEED);
		}
	}
}