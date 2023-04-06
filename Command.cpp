#include "Command.h"

//#define DEBUG

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
		RobotColl();
		//RobotAvoid();	//̽Ѱ�غ�·���������غ�·�����й滮����������Ҫ���õĻ�����Ŀ������
		RobotDoWork();
		Add_work();
		RobotSelectWork();
		Add_OK();
		Response();
		Clear();
	}
}

bool Command::ReadUntilOK()
{
	string line;
	while (getline(cin, line)) {
		if (line[0] == 'O' && line[1] == 'K')
			return true;

		#ifdef DEBUG
				cerr << line << endl;
		#endif // DEBUG


		buf.push_back(line);
	}
	return false;
}

void Command::Clear()
{
	buf.clear();
	response.clear();
	forward_s.clear();
	angle_s.clear();
	se_Need.clear();
}

void Command::Add_OK()
{
	response.push_back("OK");
}

void Command::Add_frame()
{
	response.push_back(to_string(frame));
}

void Command::Add_work()
{
	string fd = "", ro = "";
	for (const auto& [i, speed] : forward_s) {
		fd.clear();
		fd += "forward ";
		fd += to_string(i);
		fd += " ";
		fd += to_string(speed);
		response.push_back(fd);
	}
	for (const auto& [i, angle] : angle_s) {
		ro.clear();
		ro += "rotate ";
		ro += to_string(i);
		ro += " ";
		ro += to_string(angle);
		response.push_back(ro);
	}
}

void Command::Response()
{
	for (const auto& res : response) {
		cout << res << endl;
		//#ifdef DEBUG
		//		cerr << res << endl;
		//#endif // DEBUG

	}
	fflush(stdout);
}

void Command::initMap()
{
	//ѭ����ͼ���ݣ��ȱ��������й���̨�ͻ����˵ĳ�ʼλ�ã��˴�Ĭ�ϻ�����bufĩβû��ok��

	total_num = 0;
	mid_count = 0;
	map = buf;
	for (int j = buf.size() - 1; j >= 0; --j) {
		for (int i = 0; i < buf[0].size(); ++i) {
			if (buf[j][i] == 'A') {		// ������
				robot temp;
				temp.state = NONE;
				temp.object_target = make_pair(-1, -1);
				robots.push_back(temp);
			}
			if (style.find(buf[j][i] - '0') != style.end()) {	// ����̨
				if (!worker_avaliable(j, i)) {
					continue;
				}
				worker temp;
				pair<int, int> pos = make_pair(j, i);	//	����̨int����	�������ں���������ǰ����ʵ��
				pair<double, double> real_pos;			//	��������ǰ���������ں���ʵ��
				mapToreal(pos, real_pos);				//	int����תdoubleʵ������
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[j][i] - '0';
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
				idToworker[j * 100 + i] = temp;
			}
			if (buf[j][i] == '#') {
				obcTot.push_back(j * 100 + i);
			}
		}
	}

	for (const auto& [id, wo] : idToworker) {
		if (wo.style > 3 && wo.style <= 6) {
			++total_num;
		}
	}
#ifdef DEBUG
	cerr << "����̨����" << idToworker.size() << endl;
#endif // DEBUG


	robots_coll_map.resize(robots.size());
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
			/*if (end.style > 7 && start.style < 7) {
				continue;
			}*/
			double distance = 0;					//����̨֮��ľ���
			vector<int> way = can_reach(start, end, distance);		//start�ڵ��������end�ڵ�Ϳ��Լ�����BFS��
			if (way.empty()) {
				continue;
			}
			route temp(start_id, end_id);
			temp.base = start.need_money;
			temp.value = start.sell_money - start.need_money;
			temp.object = start.product_object;
			temp.length = distance;
			temp.stat = NO_PRODUCT;
			temp.line = way;
			unavaliable.push_back(temp);
		}
	}
}

void Command::UpdateInfo()
{
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

	money = m_temp;
	flush_money_stat();

	//	���ڶ���
	a = buf[index++];

	stringstream sb(a);
	sb >> worker_num;

	int n = worker_num;
	int over_num = 0;
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
		int id = tmp.pos.first * 100 + tmp.pos.second;
		worker& temp_w = idToworker[id];

		int time;
		s_for >> time;		// ����̨ʣ�������ʱ�䣨û���õ���
		temp_w.time = time;
		s_for >> temp_w.hold_object;
		s_for >> temp_w.output;


		// tmp2��¼�˴˹���̨��һ֡��״̬������û���뵱ǰ֡�Ƚ��д�
		if (temp_w.output) {
			takeoff_product_stat(id);
		}
		if (temp_w.style > 3 && temp_w.output) {
			over_num++;
		}
	}


	if (over_num > (total_num * OVER)) {
		stat = MIDP_OVER;
	}
	else {
		stat = NORMAL;
	}

	bool flag = true;

	for (int i = 0; i < robots.size(); i++) {
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

		double a_speed, face;
		pair<double, double> l_speed, real_pos;
		s_for >> a_speed >> l_speed.first >> l_speed.second;
		s_for >> face;
		s_for >> real_pos.first >> real_pos.second;

		//robots[i].on_job = item > 0;	// Я����Ʒ��Ŵ���0�ͱ�ʾ�ڹ�����
		robots[i].face = face;			// ���»�������Ϣ
		robots[i].a_speed = a_speed;
		robots[i].l_speed = l_speed;
		robots[i].real_pos = real_pos;
		robots[i].coll_angle = 0;
		robots[i].can_buy = false;
		robots[i].can_sell = false;
		robots[i].on_side = false;
		if (item != 0) {
			robots[i].hold_some = true;
		}
		else {
			robots[i].hold_some = false;
		}

		if (robots[i].on_job) {
			if (robots[i].state == BEFORE && isNear(robots[i].real_pos, idToworker[robots[i].cur.start].real_pos, 0.4)) {
				robots[i].can_buy = true;
			}
			if (robots[i].state == AFTER && isNear(robots[i].real_pos, idToworker[robots[i].cur.end].real_pos, 0.4)) {
				robots[i].can_sell = true;
			}
		}

		if (robots[i].on_coll && frame - robots[i].coll_count >= coll_frame) {
			robots[i].on_coll = false;
			robots_coll_map[i] = -1;
		}

		if (robots[i].state == AFTER && item == 0) {
			robots[i].state = BEFORE;
		}

		//Ԥ��λ��
		robots[i].n_pos.first = robots[i].l_speed.first * (PREDICT / SECONDTOT);
		robots[i].n_pos.second = robots[i].l_speed.second * (PREDICT / SECONDTOT);
		robots[i].r_speed = sqrt(l_speed.first * l_speed.first + l_speed.second * l_speed.second);

		robots[i].on_avoid = false;
		robots[i].avoid_index = 9999;
	}

	for (int i = 0; i < robots.size(); ++i) {
		double distance = 999;
		int index = i;
		for (int j = 0; j < robots.size(); ++j) {
			if (i == j || robots[i].on_coll) {
				continue;
			}
			if (IsOnmyway(robots[j], robots[i], COLL_ANGLE)) {
				if (robots_has_obc(robots[i], robots[j])) {
					continue;
				}
				robots[i].on_coll = true;
				robots[i].coll_count = frame;
				robots[i].coll_num_hide |= (1 << (j + 1));
				double dis_temp = GetLength(robots[i].real_pos, robots[j].real_pos);
				if (dis_temp < distance) {
					distance = dis_temp;
					index = j;
				}
			}
			else {
				robots[i].coll_num_hide &= (~(1 << (j + 1)));
			}
			robots_coll_map[i] = distance > COLL_RADIUS ? robots_coll_map[i] : index;
		}
	}

	takeoff_need_stat();
	flush_list();
}

void Command::RobotDoWork()
{
	for (int i = 0; i < robots.size(); ++i) {
		robot& rt = robots[i];

		if (!rt.on_job || rt.on_coll) {
			continue;
		}

		if (rt.can_buy && rt.state == BEFORE) {	//	�ܹ��򣨿���Ŀ�깤��̨��
			string temp = "";
			temp += "buy ";
			temp += to_string(i);
			response.push_back(temp);

			robots[i].state = AFTER;
			robots[i].can_buy = false;

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
			robots[i].object_target = make_pair(-1, -1);

			worker& end_worker = idToworker[robots[i].cur.end];
#ifdef DEBUG
			cerr << end_worker.time << " " << i << endl;
			cerr << (end_worker.hold_object & robots[i].cur.object) << " " << end_worker.need_object << endl;
#endif // DEBUG

			puton_need_stat(rt.cur.end, rt.cur.object);
			if ((end_worker.hold_object | robots[i].cur.object) == end_worker.need_object && end_worker.time == -1) {
				end_worker.hold_object = 0;
				takeoff_need_stat();
			}
		}

		caculate_robotPos(rt);
	}

	//ѭ���ж���Щrobot�Ѿ���ѡ����·��
	for (int i = 0; i < robots.size(); ++i) {
		robot& rt = robots[i];
		//
		//		if (!rt.on_job || rt.on_coll) {
		//			continue;
		//		}
		//
		//		if (rt.can_buy && rt.state == BEFORE) {	//	�ܹ��򣨿���Ŀ�깤��̨��
		//			string temp = "";
		//			temp += "buy ";
		//			temp += to_string(i);
		//			response.push_back(temp);
		//
		//			robots[i].state = AFTER;
		//			robots[i].can_buy = false;
		//
		//			puton_product_stat(rt.cur.start);
		//		}
		//
		//		if (rt.can_sell && rt.state == AFTER) {	//	�ܳ���
		//			string temp = "";
		//			temp += "sell ";
		//			temp += to_string(i);
		//			response.push_back(temp);
		//
		//			robots[i].state = NONE;
		//			robots[i].can_sell = false;
		//			robots[i].on_job = false;
		//			robots[i].object_target = make_pair(-1, -1);
		//
		//			worker& end_worker = idToworker[robots[i].cur.end];
		//#ifdef DEBUG
		//			cerr << end_worker.time << " " << i << endl;
		//			cerr << (end_worker.hold_object & robots[i].cur.object) << " " << end_worker.need_object << endl;
		//#endif // DEBUG
		//
		//			puton_need_stat(rt.cur.end, rt.cur.object);
		//			if ((end_worker.hold_object | robots[i].cur.object) == end_worker.need_object && end_worker.time == -1) {
		//				end_worker.hold_object = 0;
		//				takeoff_need_stat();
		//			}
		//		}
		//
		//		caculate_robotPos(rt);
		if (!rt.on_job || rt.on_coll) {
			continue;
		}

		switch (rt.state) {
		case BEFORE:
			caculate_nextWay(rt, true);
			break;
		case AFTER:
			caculate_nextWay(rt, false);
			break;
		case NONE:
			continue;
		}

		//RobotAvoid(i);

#ifdef DEBUG
		cerr << "������" << i << " " << rt.object_target.first << " " << rt.object_target.second << " "<< rt.face << endl;
#endif // DEBUG

		double speed, angle;
		normal_caculate(rt, speed, angle);

		if (GetLength(rt.object_target,rt.real_pos)<0.1) {	//	�о�����㸡����Ҫ��һ�£��̶����Ǹ�λ���е���
		//if (rt.object_target == rt.real_pos) {
			angle_s.push_back(make_pair(i, 0));
			forward_s.push_back(make_pair(i, 0));
		}
		else {
			angle_s.push_back(make_pair(i, angle));
			forward_s.push_back(make_pair(i, speed));
		}
	}

	//flush_money_stat();
	flush_list();
}

void Command::RobotSelectWork()
{
	for (int i = 0; i < robots.size(); i++) {
		if (robots[i].on_job) continue;	// ��ǰ������û�з��乤��

		if (stat == MIDP_OVER && mid_count == 0) {
			stat = NORMAL;
		}

		route my_route;
		unordered_map<int, double> accessible;
		Get_acc(robots[i], accessible);					//�ҳ���ǰ�����˵Ŀɴ�ڵ㣬��������
		if (!GetRoute(robots[i], my_route, i, accessible)) {
			continue;
		}
		puton_occ_stat(my_route.start);
		if (idToworker[my_route.end].style <= 7) {
			puton_occ_stat(my_route.end, my_route.object);
		}
		flush_list();

		vector<int> way = get_way(my_route.start, robots[i]);	//��ǰ�����˵����ľ����Լ�·��
		robots[i].cur = my_route;
		robots[i].on_job = true;
		robots[i].state = BEFORE;
		robots[i].before_way = way;
		robots[i].after_way = my_route.line;
#ifdef DEBUG
		cerr << "������" << i << "ѡ��" << idToworker[my_route.start].real_pos.first << " " <<
			idToworker[my_route.start].real_pos.second << " " << idToworker[my_route.end].real_pos.first
			<< " " << idToworker[my_route.end].real_pos.second << endl;
#endif // DEBUG

	}
}

void Command::RobotAvoid(int cur)
{
	//���Ȱ���ѭ���жϣ�ÿ��������һ�Σ���·�߳���Ϊ1000�����ӶȾ��Ѿ��ӽ����ޡ�
	//˼·ȷ����
	//�����ж��Լ�����Щ�����˵�·���غϣ������Լ��Ƿ���Ҫ���á�
	//����Ҫ���ã�������һ���µĺ�������������Ὣ����·����Ϊ�ϰ���Ȼ�󱾻�������ȥbfsѰ��·��.
	//ע�⣬������Χ��9�����Ӳ��������á�

	robot& check = robots[cur];

	unordered_set<int> avoid_wait;
	for (int i = 0; i < robots.size(); ++i) {
		if (i == cur || !robots[i].on_job) {
			continue;
		}
		const robot& target = robots[i];

		bool flag = check_avoid(check, target);
		if (flag && (check.rate == target.rate ? cur < i : check.rate < target.rate)) {
			avoid_wait.insert(i);
		}
	}

#ifdef DEBUG
	for (const auto& n : avoid_wait) {
		cerr << n << " ";
	}
	cerr << endl;
#endif // DEBUG

	GetNewWay(cur, avoid_wait);
}

void Command::RobotColl()
{
	double hold_base = M_PI, no_hold_base = M_PI_2;
	unordered_set<int> visit;
	for (int i = 0; i < robots.size(); ++i) {
		if (!robots[i].on_coll || visit.find(i) != visit.end()) {
			continue;
		}

		double angle, speed, trash;
		int another = robots_coll_map[i];
		robot target = robots[robots_coll_map[i]], cur = robots[i];
		double base = (cur.state == AFTER ? hold_base : no_hold_base);
		coll_angle_caculate(target.real_pos, cur.real_pos, target.face, cur.face, angle, base);
		normal_caculate(cur, speed, trash);
		if (i == robots_coll_map[another]) {
			double d_angle = 0, d_speed = 0;
			double base = (target.state == AFTER ? hold_base : no_hold_base);
			coll_angle_caculate(cur.real_pos, target.real_pos, cur.face, target.face, d_angle, base);
			normal_caculate(target, d_speed, trash);

			visit.insert(another);
			if (angle * d_angle < 0) {
				angle_s.push_back(make_pair(another, -d_angle));
			}
			else {
				angle_s.push_back(make_pair(another, d_angle));
			}
			if (d_speed < 3) {
				forward_s.push_back(make_pair(another, d_speed));
			}
		}
		angle_s.push_back(make_pair(i, angle));
		if (speed < 3) {
			forward_s.push_back(make_pair(i, speed));
		}
		visit.insert(i);
	}
}

bool Command::GetRoute(const robot& rb, route& ret, int id, const unordered_map<int, double>& accessible)
{
	//	Ѱ��maxValue��maxDistance�������һ���Լ�����
	double maxValue = 0, maxDistance = 0, score = 0;
	for (const auto& cur_route : avaliable) {
		if (!can_select(cur_route, accessible)) {
			continue;
		}
		route_caculate(cur_route, rb, accessible, maxValue, maxDistance, score, true, id);
	}

	for (const auto& cur_route : maybe_avaliable) {
		if (!can_select(cur_route, accessible)) {
			continue;
		}
		if (idToworker[cur_route.start].time <= 0) {
			continue;
		}
		route_caculate(cur_route, rb, accessible, maxValue, maxDistance, score, true, id);
	}
	//	����һ��������֮���ָ���Ȩ���ָ�·�ߣ�Ȼ��Ž�priority_queue

	auto cmp = [](const pair<double, route>& lhs, const pair<double, route>& rhs) {
		return lhs.first > rhs.first;
	};
	priority_queue<pair<double, route>, vector<pair<double, route>>, decltype(cmp)> pq(cmp);	//	�������double����


	for (const auto& cur_route : avaliable) {
		if (!can_select(cur_route, accessible)) {
			continue;
		}
		if (!route_caculate(cur_route, rb, accessible, maxValue, maxDistance, score, false, id)) {
			continue;
		}
		pq.push(make_pair(score, cur_route));
	}

	for (const auto& cur_route : maybe_avaliable) {
		if (!can_select(cur_route, accessible)) {
			continue;
		}
		if (idToworker[cur_route.start].time <= 0) {
			continue;
		}
		if (!route_caculate(cur_route, rb, accessible, maxValue, maxDistance, score, false, id)) {
			continue;
		}
		pq.push(make_pair(score, cur_route));	//ѡ�е�·�߸������֣�ԽСԽ�ã�����	
	}

	if (pq.empty()) {
		return false;
	}

	ret = pq.top().second;
	return true;
}

void Command::caculate_nextWay(robot& rb, bool is_before)
{
	const vector<int>& cur_way = rb.state == BEFORE ? rb.before_way : rb.after_way;

	//����ǰ����ǰ��Ŀ��ص����û�е���Ŀ��ص�
	//�о�����abs(-1 - rb.object_target.first) > 1e-6	��������ǲ��Ǻ��������
	if ( GetLength(rb.real_pos, rb.object_target) >= 0.4 && obc_check(rb.real_pos, rb.object_target)) {
		//#ifdef DEBUG
		//		cerr << rb.object_target.first << " " << rb.object_target.second << " " << rb.real_pos.first << " " << rb.real_pos.second << endl;
		//#endif // DEBUG

		return;
	}

	/*int index = -1;
	double dis = 9999;
	for (int i = 0; i < cur_way.size(); ++i) {
		int x = cur_way[i] / 100, y = cur_way[i] % 100;
		pair<double, double> dis_temp;
		mapToreal(make_pair(x, y), dis_temp);
		double cur_dis = GetLength(dis_temp, rb.real_pos);
		if (cur_dis < dis) {
			dis = cur_dis;
			index = i;
		}
	}
	rb.distanceWindex = dis;*/

	int back_ind = rb.way_index;
	if (rb.distanceWindex >= POSCHARGE) {
		// Ӧ��һ���ܵ����λ�ã���Ը��Ϊ�ع�·��
		back_ind = cur_way.size() - 1;
		for (; back_ind >= 0; --back_ind) {
			if (is_noneObc(cur_way[back_ind], rb)) {
				idToreal(cur_way[back_ind], rb.object_target);
				break;
			}
		}
	}
	else {
		idToreal(cur_way[back_ind + 1], rb.object_target);
		for (int i = cur_way.size() - 1; i > back_ind; --i) {
			if (is_noneObc(cur_way[i], rb)) {
				idToreal(cur_way[i], rb.object_target);
				break;
			}
		}
	}
}

// ȡ��ǰ��
void Command::idTomap(int id, pair<int, int>& res) {
	int x = id / 100, y = id % 100;
	res.first = x;
	res.second = y;
}

void Command::idToreal(int id, pair<double, double>& res) {
	pair<int, int> old = make_pair(0, 0);
	Command::idTomap(id, old);
	Command::mapToreal(old, res);
}

void Command::mapToreal(pair<int, int> old, pair<double, double>& ret)
{
	int temp_x = old.second;
	int temp_y = old.first;
	ret.first = static_cast<double>(temp_x) / 2 + 0.25;
	ret.second = 49.75 - (static_cast<double>(temp_y) / 2);
	//0.5 * j + 0.25, 49.75 - 0.5 * i
}

// �������
void Command::realTomap(pair<double, double> old, pair<int, int>& ret)
{
	double temp_y = old.second;
	double temp_x = old.first;
	ret.first = (temp_y - 0.25) * 2;
	ret.second = (temp_x - 0.25) * 2;
	ret.first = map.size() - 1 - ret.first;
}

//	��������֮��ľ���
double Command::GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs) {
	double x = lhs.first - rhs.first, y = lhs.second - rhs.second;
	return sqrt(x * x + y * y);
}

// �ж�����λ�������Ƿ�С��dis
bool Command::isNear(const pair<double, double>& lhs, const pair<double, double>& rhs, double dis)
{
	if (GetLength(lhs, rhs) <= dis) {
		return true;
	}
	return false;
}

// �жϻ�������û���ڶԷ�·����
bool Command::IsOnmyway(const robot& target, const robot& check, double pi)
{
	double x = target.real_pos.second - check.real_pos.second,
		y = (target.real_pos.first - check.real_pos.first);
	double target_angle = atan2(x, y);	//	����Ŀ��Ƕ���x������н�
	double a_diff = target_angle + (-check.face);	//	����Ŀ��Ƕ��볯��Ĳ�ֵ
	if (a_diff > M_PI) {
		a_diff -= 2 * M_PI;
	}
	else if (a_diff < (-M_PI)) {
		a_diff += 2 * M_PI;
	}
	a_diff = abs(a_diff);
	if (a_diff <= pi && isNear(target.real_pos, check.real_pos, COLL_RADIUS)) {
		return true;
	}
	if ((outline_check(target, check, pi) && a_diff <= OUTLINE_RADIUS) && isNear(target.real_pos, check.real_pos, COLL_RADIUS)) {
		return true;
	}
	return false;
}

void Command::normal_caculate(const robot& rt, double& speed, double& angle)
{
	//�ҵ���Ŀ��
	double x = rt.object_target.second - rt.real_pos.second,
		y = (rt.object_target.first - rt.real_pos.first);
	double target_angle = atan2(x, y);	//	����Ŀ��Ƕ���x������н�
	double a_diff = target_angle + (-rt.face);	//	����Ŀ��Ƕ��볯��Ĳ�ֵ
	double distance = GetLength(rt.object_target, rt.real_pos);

	if (a_diff > M_PI) {
		a_diff -= 2 * M_PI;
	}
	else if (a_diff < (-M_PI)) {
		a_diff += 2 * M_PI;
	}
	int dir = a_diff > 0 ? 1 : -1;
	a_diff = abs(a_diff);

	//const double maxRotateSpeed = (dir > 0 ? M_PI : -M_PI);     //���ٶ�����
	//const double maxSpeed = min(distance / 0.0525, 6.);             //Ŷ�𣿺�����˼����ֵmarkmark
	//if (a_diff < M_PI_32) { // ��������Ŀ���ļнǺ�С��ֱ��ȫ��ǰ��
	//	speed = 6;
	//	angle = 0;
	//}
	//else {
	//	if (a_diff > M_PI_2) {
	//		// �Ƕ�̫��ȫ��Ťת
	//		// �ٶȿ���Сһ�㣬���⿿�����˹���̨
	//		speed = 6.0 * 0.2;
	//		angle = maxRotateSpeed;
	//	}
	//	else {
	//		speed = 6. * cos(a_diff); // ǰ���ٶ���Ƕȱ�С�����
	//		angle = maxRotateSpeed * sin(a_diff);    // ��ת�ٶ���Ƕȱ�С����С
	//	}
	//}

	//�������˺�Ŀ��ص�ƫ�Ǻܴ�
	if (a_diff >= M_PI_2) {
		speed = -2;
		angle = M_PI * dir;
		if (is_same_face(rt)) {
			angle = 0;
		}
	}
	//�������˺�Ŀ��ص�ƫ�ǽϴ�
	else if (a_diff >= M_PI_8) {
		speed = 1;
		angle = M_PI * dir;
	}
	//��΢��  
	else if (a_diff >= M_PI_32) {
		speed = 6;
		angle = M_PI_4 * dir;
	}
	else if (a_diff >= M_PI_64) {
		speed = 6;
		angle = M_PI_8 * dir;
	}
	//ǰ��
	else {
		speed = 6;
		angle = 0;
	}

	/*if (distance < 0.4) {
		speed = 0.1;
		return;
	}*/

	target_slowdown(rt, speed, angle);

	//#ifdef DEBUG
	//	cerr << rt.real_pos.first << " " << rt.real_pos.second << " " << speed << " " << angle << endl;
	//#endif // DEBUG


		/*double obc_dis = 9999;
		for (const auto& pos_id : obcTot) {
			int x = pos_id / 100;
			int y = pos_id % 100;
			pair<double, double> obc_temp;
			mapToreal(make_pair(x, y), obc_temp);
			obc_dis = min(obc_dis, GetLength(obc_temp, rt.real_pos));
		}
		if (obc_dis <= OBCMINDIS) {
			speed *= 0.2;
		}*/
}

// ����ǲ�ֵ
void Command::coll_angle_caculate(const pair<double, double>& target, const pair<double, double>& cur, const double& t_face, const double& c_face, double& angle, double base)
{
	double x = target.second - cur.second,
		y = (target.first - cur.first);
	double target_angle = atan2(x, y);	//	����Ŀ��Ƕ���x������н�
	double a_diff = target_angle + (-c_face);	//	����Ŀ��Ƕ��볯��Ĳ�ֵ
	if (a_diff > M_PI) {
		a_diff -= 2 * M_PI;
	}
	else if (a_diff < (-M_PI)) {
		a_diff += 2 * M_PI;
	}
	int dir = a_diff > 0 ? -1 : 1;

	double face_dif = t_face - c_face;
	face_dif = abs(face_dif);

	int face_dir = face_dif <= M_PI_2 ? -1 : 1;

	angle = base * dir * (face_dif > M_PI_H ? face_dir : 1);
}

// ����·��Ȩֵ����
bool Command::route_caculate(const route& cur_route, const robot& rb, const unordered_map<int, double>& accessible, double& maxValue, double& maxDistance, double& score, bool is_first, int id)
{
	worker route_start_worker = idToworker[cur_route.start];
	worker route_end_worker = idToworker[cur_route.end];
	pair<double, double> cur_worker_pos = route_start_worker.real_pos;	//û�õ��������
	double distance = accessible.at(cur_route.start);			 //	��������˵���ʼ�����
	if (cur_route.stat == NO_PRODUCT && (lengthOneFrame * route_start_worker.time) > GetLength(rb.real_pos, route_start_worker.real_pos)) {
		return false;
	}
	distance += cur_route.length;
	if (lastbuyselect * (15000 - frame) < distance) {
		return false;
	}
	if (is_first) {
		maxValue = max(maxValue, cur_route.value);
		maxDistance = max(maxDistance, distance);
	}
	else {
		distance = distance / maxDistance;	//·��ԽСԽ�ã�����һ����С��ָ��
		double temp_value;
		temp_value = 1 - cur_route.value / maxValue;	// ��ֵԽ��Խ�ã���һ��������ָ�꣬��Ҫ����
		score = temp_value * VALUE_WEIGHT + distance * LENGTH_WEIGHT;	//	��·��Ȩֵ
	}
	return true;
}

bool Command::can_select(const route& cur, const unordered_map<int, double>& accessible)
{
	worker route_start_worker = idToworker[cur.start];
	if (stat == MIDP_OVER && route_start_worker.style <= 3) {
		return false;
	}
	if (accessible.count(cur.start) == 0) {
		return false;
	}
	return true;
}

// �Ϻ���
bool Command::closeToside(const robot& cur)
{
	double x = cur.real_pos.first, y = cur.real_pos.second;
	double timex1 = abs((x - 0.5) / cur.l_speed.first), timex2 = abs((49.5 - x) / cur.l_speed.second);
	double timey1 = abs((y - 0.5) / cur.l_speed.second), timey2 = abs((49.5 - y) / cur.l_speed.second);

	if (min(timex1, timex2) <= 0.3) {
		return true;
	}
	if (min(timey1, timey2) <= 0.3) {
		return true;
	}
	return false;
}

// �������ж������Ż���ײ���
bool Command::outline_check(const robot& target, const robot& check, double pi)
{
	double k1 = tan(check.face + pi);
	double k2 = tan(check.face - pi);
	double x1 = check.real_pos.first, y1 = check.real_pos.second;
	double x2 = target.real_pos.first, y2 = target.real_pos.second;
	double dis1 = abs(k1 * (x2 - x1) - (y2 - y1)) / sqrt(k1 * k1 + 1);
	double dis2 = abs(k2 * (x2 - x1) - (y2 - y1)) / sqrt(k2 * k2 + 1);
	if (dis1 < target.state == AFTER ? 0.53 : 0.45) {
		return true;
	}
	if (dis2 < target.state == AFTER ? 0.53 : 0.45) {
		return true;
	}

	return false;
}

// �鹤��̨�Ƿ��ڽ���
bool Command::worker_avaliable(int x, int y)
{
	int total = 0, conti = 0;
	int temp = 0;
	for (int i = 0; i < 4; ++i) {
		int nx = x + dic[i][0];
		int ny = y + dic[i][1];
		++temp;
		if (nx < 0 || nx >= map.size() || ny < 0 || ny >= map[0].size() || map[nx][ny] == '#') {
			++total;
			conti = max(temp, conti);
		}
		else {
			temp = 0;
		}
	}
	switch (total) {
	case 0:
	case 1:
		return true;
	case 2:
		if ((int)(map[x][y] - '0') > 3 || conti == 1) {
			return false;
		}
	case 3:
	case 4:
		return false;
	}
}

// is_range�ж��жϵ��Ƿ��ڵ�ͼ��
bool Command::is_range(int x, int y)
{
	if (x >= 0 && x < map.size() && y >= 0 && y < map[0].size()) {
		return true;
	}
	return false;
}

void Command::Get_acc(const robot& rb, unordered_map<int, double>& accessible)
{
	int x, y;
	get_closePoint(rb, x, y);	//	�ҳ����������ĸ�����

	queue<pair<int, int>> qe;	//	BFS ��·
	qe.push(make_pair(x, y));
	unordered_set<int> visit;
	visit.insert(x * 100 + y);
	int dep = 0;
	while (!qe.empty()) {
		int length = qe.size();
		++dep;
		while (length--) {
			auto cur = qe.front();
			qe.pop();
			for (int i = 0; i < 4; ++i) {
				int nx = cur.first + dic[i][0];
				int ny = cur.second + dic[i][1];
				if (is_range(nx, ny) && map[nx][ny] != '#' && visit.find(nx * 100 + ny) == visit.end()) {
					if (!test_side(i, cur.first, cur.second, true)) {
						continue;
					}
					if (style.find(map[nx][ny] - '0') != style.end()) {
						accessible[nx * 100 + ny] = dep;
					}
					visit.insert(nx * 100 + ny);
					qe.push(make_pair(nx, ny));
				}
			}
		}
	}
}

void Command::get_closePoint(const robot& rb, int& x, int& y)
{
	double distance = 1000000;

	// �о�������԰�rb����ȡ���Ż��£����Ǳ߽��ж��е��鷳���Ȳ�д��
	//int rb_x = int(rb.real_pos.first);
	//int rb_y = int(rb.real_pos.second);
	for (int i = 0; i < map.size(); ++i) {
		for (int j = 0; j < map[0].size(); ++j) {
			pair<int, int> pos = make_pair(i, j);
			pair<double, double> real;
			mapToreal(pos, real);
			double temp = GetLength(real, rb.real_pos);
			if (temp < distance) {
				distance = temp;
				x = i, y = j;
			}
		}
	}
}

//���ϰ��ﷵ��false;
bool Command::is_noneObc(int id, const robot& rb)
{
	pair<double, double> temp;
	idToreal(id, temp);
	return obc_check(temp, rb.real_pos);
}

// �ж��ڲ��ڵ�ͼ�߽�����Ƿ����ϰ���
bool Command::test_side(int step, int x, int y, bool is_before)
{
	int nx = x + dic[step][0], ny = y + dic[step][1];
	int count = 0;
	for (int i = 0; i < 4; ++i) {
		int kx = nx + dic[i][0];
		int ky = ny + dic[i][1];
		if (!is_range(kx, ky) || map[kx][ky] == '#') {
			++count;
		}
	}
	/*if (is_before) {
		int kx = nx + dic[step][0], ky = ny + dic[step][1];
		if (count <= 1 && (is_range(kx, ky) && map[kx][ky] != '#')) {
			return true;
		}
		return false;
	}*/
	if (count) {
		return false;
	}
	return true;
}

bool Command::is_same_face(const robot& rb)
{
	double theta = rb.face;
	if (theta >= 0 && theta <= M_PI_2) {
		if (rb.l_speed.first >= 0 && rb.l_speed.second >= 0) {
			return true;
		}
	}
	else if (theta >= M_PI_2 && theta <= M_PI) {
		if (rb.l_speed.first <= 0 && rb.l_speed.second >= 0) {
			return true;
		}
	}
	else if (theta <= 0 && theta >= -M_PI_2) {
		if (rb.l_speed.first >= 0 && rb.l_speed.second <= 0) {
			return true;
		}
	}
	else if (theta <= -M_PI_2 && theta >= -M_PI) {
		if (rb.l_speed.first <= 0 && rb.l_speed.second <= 0) {
			return true;
		}
	}
	return false;
}

//���ϰ��ﷵ��true��û�з���false
bool Command::robots_has_obc(const robot& lhs, const robot& rhs)
{
	return !obc_check(lhs.real_pos, rhs.real_pos);
}

//���ϰ��ﷵ��false
bool Command::obc_check(const pair<double, double>& lhs, const pair<double, double>& rhs)
{
	double x1 = lhs.first, y1 = lhs.second;
	double x2 = rhs.first, y2 = rhs.second;
	double t_x = y1 - y2;
	double t_y = x1 - x2;
	double theta = atan2(t_x, t_y);
	double k = tan(theta);

	for (const auto& id : obcTot) {
		int x = id / 100, y = id % 100;
		pair<double, double> temp;
		mapToreal(make_pair(x, y), temp);

		if (!(temp.first <= max(x2, x1) + OBCMINDIS && temp.first >= min(x2, x1) - OBCMINDIS
			&& temp.second <= max(y2, y1) + OBCMINDIS && temp.second >= min(y2, y1) - OBCMINDIS)) {
			continue;
		}

		double dis = abs(k * (temp.first - x2) - (temp.second - y2)) / sqrt(k * k + 1);
		if (dis <= OBCMINDIS) {
			return false;
		}
	}

	return true;

	//double x1 = lhs.first, y1 = lhs.second;
	//double x2 = rhs.first, y2 = rhs.second;
	//double distance = GetLength(lhs, rhs);
	//	��ֱ��
	//double k = (y1 - y2) / (x1 - x2);
	//for (const auto& num : obcTot) {
	//	pair<double, double> real;
	//	idToreal(num, real);
	//	double dis1 = GetLength(real, lhs);
	//	double dis2 = GetLength(real, rhs);
	//	if (dis1 > distance || dis2 > distance) {	// �ж��ϰ����ڻ�����������
	//		continue;
	//	}
	//	double x0 = real.first, y0 = real.second;
	//	double dis3 = abs(k * (x0 - x1) - y0 + y1) / (sqrt(k * k + 1));	// �ϰ���㵽�����������߾���
	//	if (dis3 < OBCMINDIS) {
	//		return true;
	//	}
	//}
	//return false;
}

void Command::target_slowdown(const robot& rt, double& speed, double& angle)
{
	/*pair<double, double> target = rt.object_target;
	pair<double, double> real_pos = rt.real_pos;
	double x = GetLength(target, real_pos);
	double y = 6.0 / (1 + exp((-log(11) * x / 3 + log(11))));
	int dir = speed >= 0 ? 1 : -1;
	speed = y * dir;*/

	pair<double, double> target = rt.object_target;
	pair<double, double> real_pos = rt.real_pos;
	double x = GetLength(target, real_pos);
	double y = 4 * (1 - exp(-0.5413 * x)) + 2;
	int dir = speed >= 0 ? 1 : -1;
	speed = y * dir;
}

void Command::caculate_robotPos(robot& rb)
{
	vector<int> cur_way;
	if (rb.state == BEFORE) {
		cur_way = rb.before_way;
	}
	else {
		cur_way = rb.after_way;
	}

	int index = -1;
	double dis = 9999;
	for (int i = 0; i < cur_way.size(); ++i) {
		pair<double, double> dis_temp;
		idToreal(cur_way[i], dis_temp);
		double cur_dis = GetLength(dis_temp, rb.real_pos);
		if (cur_dis < dis) {
			dis = cur_dis;
			index = i;
		}
	}
	rb.way_index = index;
	rb.distanceWindex = dis;
	rb.rate = static_cast<double>(index) / cur_way.size();
}

void Command::GetNewWay(int index, const unordered_set<int>& avoid_wait)
{
	if (avoid_wait.empty()) {
		return;
	}
	vector<string> copy_map = map;

	//������·
	for (const auto& ind : avoid_wait) {
		const auto& cur_way = robots[ind].state == BEFORE ? robots[ind].before_way : robots[ind].after_way;
		for (int start = robots[ind].way_index; start < cur_way.size(); ++start) {
			pair<int, int> pos;
			pair<double, double> p1, p2;	// p1��ǰһ��λ��
			p1 = p2;
			idToreal(start, p2);
			idTomap(cur_way[start], pos);
			if (!worker_exist(pos)) {
				map[pos.first][pos.second] = '#';
			}
			if (!obc_check(p2, robots[ind].real_pos)) {
				robots[ind].object_target = p1;	// �ӵ�ǰλ����·���յ����������һ��;�����ϰ���ģ���ǰһ����Ϊobj_target
			}
		}
	}

	int target = (robots[index].state == BEFORE ? robots[index].cur.start : robots[index].cur.end);
	auto& cur_way = robots[index].state == BEFORE ? robots[index].before_way : robots[index].after_way;
	auto way = get_way(target, robots[index]);
	if (way.empty()) {
		robots[index].on_avoid = true;
		//robots[index].object_target = robots[index].real_pos;	// Ϊʲô����Ҫ��ôд����
	}
	else {
		cur_way = way;    
		robots[index].way_index = 0;
		caculate_nextWay(robots[index], robots[index].state == BEFORE);
	}

	map = copy_map;
}

//�����Χ�˸��й���̨����true��û�з���false
bool Command::worker_exist(const pair<int, int>& cur)
{
	for (int i = 0; i < eight.size(); ++i) {
		int nx = cur.first + eight[i][0];
		int ny = cur.first + eight[i][1];
		if (is_range(nx, ny) && isdigit(map[nx][ny])) {
			return true;
		}
	}
	return false;
}

//·���н��淵��true�����򷵻�false
bool Command::check_avoid(robot& check, const robot& target)
{
	const vector<int>& cur_way = check.state == BEFORE ? check.before_way : check.after_way;
	const vector<int>& t_way = target.state == BEFORE ? check.before_way : check.after_way;
	
	int lhs = check.way_index, rhs = target.way_index;

	for (int i = lhs; i < cur_way.size(); ++i) {
		for (int j = rhs; j < t_way.size(); ++j) {
			pair<double, double> templ, tempr;
			idToreal(cur_way[i], templ);
			idToreal(t_way[i], tempr);
			if (GetLength(templ, tempr) <= ROBMINDIS) {
				check.avoid_index = min(check.avoid_index, i);
				return true;
			}
		}
	}

	return false;
}

vector<int> Command::can_reach(const worker& start, const worker& end, double& distance)
{
	distance = 0;
	return BFS(start.pos, end.pos, distance, false);
}

vector<int> Command::get_way(int id, const robot& rb)
{
	int x, y;
	get_closePoint(rb, x, y);

	int t_x = id / 100, t_y = id % 100;
	double temp;
	return BFS(make_pair(x, y), make_pair(t_x, t_y), temp, true);
}

//	���avaliable������unavaliable
void Command::Clean_list() {
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		unavaliable.insert(unavaliable.end(), *p);
		p = avaliable.erase(p);
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		unavaliable.insert(unavaliable.end(), *p);
		p = maybe_avaliable.erase(p);
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		p->stat = NO_PRODUCT;
		if ((idToworker[p->end].hold_object & p->object) != 0) {
			p->stat |= NO_NEED;
		}
	}
}

//	���±�ѡ�е�·��
void Command::flush_list() {
	mid_count = 0;
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			maybe_avaliable.insert(maybe_avaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else {
			if (p->object > OBJECT_THREE) {
				++mid_count;
			}
			++p;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			++p;
		}
		else if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = maybe_avaliable.erase(p);
		}
		else {
			avaliable.insert(avaliable.end(), *p);
			p = maybe_avaliable.erase(p);
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			maybe_avaliable.insert(maybe_avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
		else if (p->stat != 0) {
			++p;
		}
		else {
			avaliable.insert(avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
	}
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

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
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

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~NO_PRODUCT);
		}
	}
}

void Command::puton_product_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC_S);
			p->stat |= NO_PRODUCT;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC_S);
			p->stat |= NO_PRODUCT;
		}
	}
}

//start�汾
void Command::puton_occ_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}
}

//end�汾
void Command::puton_occ_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}
}

void Command::puton_need_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
			p->stat |= NO_NEED;
		}
	}
}

vector<int> Command::BFS(const pair<int, int>& start, const pair<int, int>& end, double& distance, bool is_before)
{
	queue<tuple<int, int, vector<int>>> qe;
	bool flag = false;
	vector<int> ret;
	vector<int> temp{ start.first * 100 + start.second };
	unordered_set<int> visit;

	qe.push(make_tuple(start.first, start.second, temp));
	visit.insert(start.first * 100 + start.second);
	while (!qe.empty()) {
		int length = qe.size();
		while (length--) {
			auto cur = qe.front();
			qe.pop();
			int x = get<0>(cur), y = get<1>(cur);
			if (x == end.first && y == end.second) {
				flag = true;
				ret = move(get<2>(cur));
				break;
			}
			for (int i = 0; i < 4; ++i) {
				int nx = x + dic[i][0];
				int ny = y + dic[i][1];
				if (is_range(nx, ny) && map[nx][ny] != '#' && visit.find(nx * 100 + ny) == visit.end()) {
					if (!test_side(i, x, y, is_before)) {
						continue;
					}
					visit.insert(nx * 100 + ny);
					vector<int> next = get<2>(cur);
					next.push_back(nx * 100 + ny);
					qe.push(make_tuple(nx, ny, next));
				}
			}
		}
		if (flag) {
			break;
		}
	}

	if (flag) {
		distance = ret.size();
		return ret;
	}
	return {};
}

void Command::takeoff_need_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if ((idToworker[p->end].hold_object & p->object) == 0) {
			p->stat &= (~NO_NEED);
		}
	}
}