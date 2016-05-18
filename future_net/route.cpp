#include "route.h"
#include "lib_record.h"
#include <stdio.h>
using namespace std;

vector<Edge> allEdges;		//边的集合
#ifdef TIME
clock_t arr[10];
#endif
Path &Path::operator=(const Path &rhs)
{
#ifdef TIME
	clock_t starttime = clock();
#endif
	cost = rhs.cost;
	visited = rhs.visited;
	start = rhs.start;
	target = rhs.target;
	edges = rhs.edges;
	spNum = rhs.spNum;
	i = rhs.i;
	j = rhs.j;
	k = rhs.k;
#ifdef TIME
	clock_t endtime = clock();
	arr[0] += endtime - starttime;
#endif
	return *this;
}

Path Path::operator+(const Path &rhs)
{
#ifdef TIME
	clock_t starttime = clock();
#endif
	Path sum = *this;
	sum.cost = this->cost + rhs.cost;
	sum.visited = rhs.visited;
	sum.spNum += rhs.spNum - 1;

	for(unsigned int i = 0; i < sum.edges.size(); i++) {
		if(i == 0) sum.visited.set(allEdges[sum.edges[i]].to);
		sum.visited.set(allEdges[sum.edges[i]].from);
	}
	if(sum.i == -1 && rhs.i == -1) {
		vector<int> tmp = sum.edges;
		sum.edges = rhs.edges;
		for(unsigned int i = 0; i < tmp.size(); i++)
			sum.edges.push_back(tmp[i]);
		sum.i = -2;
	}
#ifdef TIME
	clock_t endtime = clock();
	arr[1] += endtime - starttime;
#endif
	return sum;
}

void Path::handleCycle(Path &rhs, Dijkstra &work, int WorB)
{
#ifdef TIME
	clock_t starttime = clock();
#endif
	bool hasCycle = false;	
	if(this->target == rhs.start) {	
		for(unsigned int i = 0; i < this->edges.size(); i++) {
			if(rhs.visited[allEdges[this->edges[i]].from] == true) {
				hasCycle = true;
				break;
			}
		}
	}
	else {
		for(unsigned int i = 0; i < this->edges.size(); i++) {
			if(rhs.visited[allEdges[this->edges[i]].to] == true) {
				hasCycle = true;
				break;
			}
		}
	}
	if(hasCycle == false) {
#ifdef TIME
		clock_t endtime = clock();
		arr[2] += endtime - starttime;
#endif
		return;
	}

	bitset<maxn> rmNodes;
	rmNodes = rhs.visited;

	if(this->target == rhs.start) rmNodes[this->target] = false;
	else rmNodes[this->start] = false;

	if(this->start != work.start) rmNodes[work.start] = true;
	if(this->target != work.target) rmNodes[work.target] = true;
	work.dijkstra_2nodes(this->start, this->target, rmNodes);
	work.BuildPath(this->start, this->target, *this);
#ifdef TIME
	clock_t endtime = clock();
	arr[2] += endtime - starttime;
#endif
}

void Dijkstra::init(char *topo[5000], int edge_num, char *demand[MAX_DEMAND_NUM]) {	//用节点数目初始化
#ifdef TIME
	clock_t starttime = clock();
#endif
	m = edge_num;
	n = 0;
	WorB = 0;

	for (int i = 0; i < maxn; i++) G[i].clear();
	allEdges.clear();

	int index, from, to, dist;
	char tmp;

	for(int i = 0; i < 2; i++) {
		if(includingSet.size() == 0) {
			includingSet.push_back({});
			includingSet.push_back({});
		}
		int index = 0;
		for(int dou = 0; dou < 3; index++) {
			if(demand[i][index] == ',') dou++;
		}
		if(demand[i][index] == 'N') continue;
		stringstream ss;
		ss << demand[i];
		ss >> tmp >> tmp;
		ss >> this->start >> tmp >> this->target >> tmp;
		do{
			ss >> from;
			includingSet[i].push_back(from);
			isIncludingSet[i][from] = true;
		}while(ss >> tmp);
	}

	for(int i = 0; i < edge_num; i++){
		stringstream ss;
		ss << topo[i];
	    	ss >> index >> tmp >> from >> tmp >> to >> tmp >> dist;
		if(from != target && to != start) this->AddEdge(index, from, to, dist);	
	}
	/*vector<vector<vector<int> > > G_init(maxn, vector<vector<int>>(maxn));
	int maxNodeNum;
	for(int i = 0; i < edge_num; i++){
		stringstream ss;
		ss << topo[i];
	    	ss >> index >> tmp >> from >> tmp >> to >> tmp >> dist;
		while(1) {
			if((int)allEdges.size() == index) {
				allEdges.push_back(Edge(index, from, to, dist));//假设边的读入顺序和index相同。否则需要事先设计allEdges的大小。
				break;
			}
			else if((int)allEdges.size() > index) {
				allEdges[index] = Edge(index, from, to, dist);
				break;
			}
			else allEdges.push_back(Edge(-1, -1, -1, -1));
		}
		maxNodeNum = max(from, to);
		this->n = max(this->n, maxNodeNum + 1);
		G_init[from][to].push_back(index);
	}

	for(int i = 0; i < n; i++) G_init[i][start].clear();
	for(int i = 0; i < n; i++) G_init[target][i].clear();
	G_init[start][target].clear();

	int sum = 0;
	bitset<maxn> done1;
	for(int i = 0; i < n; i++) {
		int inDegree = 0, outDegree = 0;
		int in, out;
		for(int j = 0; j < n; j++) {
			if(G_init[i][j].size() != 0) {
				outDegree++;
				out = j;
			}
			if(G_init[j][i].size() != 0) {
				inDegree++;
				in = j;
			}
		}
		if(inDegree == 0 && i != start) {
			for(int j = 0; j < n; j++) G_init[i][j].clear();
		}
		if(outDegree == 0 && i != target) {
			for(int j = 0; j < n; j++) G_init[j][i].clear();
		}
		if(inDegree == 1 && (isIncludingSet[0][i] || isIncludingSet[1][i])) {
			for(int j = 0; j < n; j++) if(j != i) G_init[in][j].clear();
		}
		if(outDegree == 1 && (isIncludingSet[0][i] || isIncludingSet[1][i])) {
			for(int j = 0; j < n; j++) if(j != i) G_init[j][out].clear();
		}
	}

	for(unsigned int i = 0; i < (unsigned int)n; i++) {
		for(unsigned int j = 0; j < (unsigned int)n; j++) {
			for(unsigned int k = 0; k < G_init[i][j].size(); k++) {
				G[i].push_back(G_init[i][j][k]);
			}
		}
	}*/
#ifdef TIME
	clock_t endtime = clock();
	arr[3] += endtime - starttime;
#endif
}

void Dijkstra::AddEdge(int index, int from, int to, int dist) {	//添加一条边
#ifdef TIME
	clock_t starttime = clock();
#endif
	while(1) {
		if((int)allEdges.size() == index) {
			allEdges.push_back(Edge(index, from, to, dist)); //此处假设边的读入顺序和index相同。否则需要事先设计allEdges的大小。
			break;
		}
		else if((int)allEdges.size() > index) {
			allEdges[index] = Edge(index, from, to, dist);
			break;
		}
		else allEdges.push_back(Edge(-1, -1, -1, -1));
	}
	int tmp = max(from, to);
	this->n = max(this->n, tmp + 1);
	G[from].push_back(index);
#ifdef TIME
	clock_t endtime = clock();
	arr[4] += endtime - starttime;
#endif
}

void Dijkstra::dijkstra(int s, const bitset<maxn> &rmNodes) {	//计算起点到各个点的最短路径
#ifdef TIME
	clock_t starttime = clock();
#endif
	fill(d, d + n, INT_MAX);	//初始化d数组
	d[s] = 0;

	done.reset();	//初始化done数组
	done = rmNodes;
		
	memset(p, -1, sizeof(p));	//-1表示没有这条边
	priority_queue<HeapNode> Q;
	Q.push(HeapNode(0, s));	//把起点加入队列

	while (!Q.empty()) {
		HeapNode x = Q.top(); Q.pop();
		int u = x.u;
		if (done[u]) continue;
		done[u] = true;
		for (unsigned int i = 0; i < G[u].size() && G[u][i] != -1; i++) { //遍历节点u的所有边, 删除的边不遍历
			Edge& e = allEdges[G[u][i]];
			if (d[e.to] > d[u] + e.phe) {
				d[e.to] = d[u] + e.phe;
				p[e.to] = G[u][i];
				Q.push(HeapNode(d[e.to], e.to));
			}
		}
	}
#ifdef TIME
	clock_t endtime = clock();
	arr[5] += endtime - starttime;
#endif
}

void Dijkstra::dijkstra_2nodes(int s, int t, const bitset<maxn> &rmNodes) {	//计算起点到各个点的最短路径
#ifdef TIME
	clock_t starttime = clock();
#endif	
	fill(d, d + n, INT_MAX);
	d[s] = 0;

	done.reset();	//初始化done数组
	done = rmNodes;
	memset(p, -1, sizeof(p));	//-1表示没有这条边
	priority_queue<HeapNode> Q;
	Q.push(HeapNode(0, s));	//把起点加入队列

	while (!Q.empty()) {
		HeapNode x = Q.top(); Q.pop();
		int u = x.u;
		if (done[u]) continue;
		done[u] = true;

		for (unsigned int i = 0; i < G[u].size() && G[u][i] != -1; i++) {	//	遍历节点u的所有边, 删除的边不遍历
			Edge& e = allEdges[G[u][i]];
			if (d[e.to] > d[u] + e.phe) {
				d[e.to] = d[u] + e.phe;
				p[e.to] = G[u][i];
				if (e.to == t) {
		#ifdef TIME
					clock_t endtime = clock();
					arr[7] += endtime - starttime;
		#endif
					return;
				}
				Q.push(HeapNode(d[e.to], e.to));
			}
		}
	}
#ifdef TIME
	clock_t endtime = clock();
	arr[7] += endtime - starttime;
#endif
}

void Dijkstra::BuildPath(int vi, int vl, Path& path)
{
#ifdef TIME
	clock_t starttime = clock();
#endif
	path.cost = this->d[vl];
	path.start = vi;
	path.target = vl;
	path.edges.clear();
	path.visited.reset();
	path.spNum = 0;
	path.i = -1;
	for(int i = vl; ; ){	//从终点开始
		if(find(this->includingSet[WorB].begin(), this->includingSet[WorB].end(), i) != this->includingSet[WorB].end())
			path.spNum++;
		path.visited.set(i);

		if(this->p[i] == -1) {
			path.cost = -1;
			break;	//不存在这条路径
		}
		path.edges.push_back(this->p[i]);	//找到一条边
		if((i = allEdges[this->p[i]].from) != vi) continue;
		else{
			if(i != this->start && i != this->target) path.spNum++;
			path.visited.set(i);
			break;
		}
	}
#ifdef TIME
	clock_t endtime = clock();
	arr[6] += endtime - starttime;
#endif
}

void Path::printPath(class Dijkstra &work)
{
#ifdef PRINT_PATH
	cout << "Path:" << start << "->" << target;
	cout << "cost:" << cost << endl;
	cout << "spNum:" << spNum << endl;
	cout << "Nodes:";
	for(int in = edges.size() - 1; in >= 0; in--) {
		if(in == (int)edges.size() - 1) cout << allEdges[edges[in]].from << "->" << allEdges[edges[in]].to;
		else cout << "->" << allEdges[edges[in]].to;
	}
	cout << endl;
	cout << "cant table:";
	for(int in = 0; in < work.n; in++) {
		if(visited[in] == true) cout << in << " ";
	}
	cout << endl;
	cout << "pre: " << i << ' ' << j << ' ' << k << endl; 
	cout << endl;
#endif
}

void okPath::printPath()
{
	cout << "cost:" << cost << endl;
	cout << "edges:";
	for(unsigned int i = 0; i < edges.size(); i++) {
		if(i != 0) cout << "|" << edges[i];
		else cout << edges[i];
	}
	cout << endl;
}

void okPath::judgePath(class Dijkstra &work, int WorB)
{
	if(allEdges[edges[0]].from != work.start) {
		cout << "start point is wrong" << endl;
		return;
	}
	if(allEdges[edges[edges.size() - 1]].to != work.target) {
		cout << "target point is wrong" << endl;
		return;
	}	
	bitset<maxn> visited; //判断环
	bitset<maxn> visitedsp; //判断spNodes
	visited[allEdges[edges[0]].from] = true;
	cout << "point:";
	for(unsigned int i = 0; i < edges.size(); i++) {
		if(i == 0) cout << allEdges[edges[i]].from << "->" << allEdges[edges[i]].to;
		else cout << "->" << allEdges[edges[i]].to;
		if(visited[allEdges[edges[i]].to] == true) {
			cout << "cycle:" << allEdges[edges[i]].to << endl;
			return;
		}
		else visited[allEdges[edges[i]].to] = true;
		if(work.isIncludingSet[WorB][allEdges[edges[i]].to] == true) visitedsp[allEdges[edges[i]].to] = true;
	}
	cout << endl;
	if(visitedsp.count() < work.includingSet[WorB].size()) {
		cout << "unreached node" << endl;
		return;
	}
	cout << "path is correct" << endl;
}
	
void okPath::changeCost()
{
	int cost = 0;
	for(unsigned int i = 0; i < edges.size(); i++) cost += allEdges[edges[i]].dist;
	this->cost = cost;
}

int repeatEdges(vector<int> &workpath, vector<int> &backpath, vector<int> &edges, bool makeEdges)
{
	int repeatNum = 0;
	for(unsigned int i = 0; i < workpath.size(); i++) {
		if(find(backpath.begin(), backpath.end(), workpath[i]) != backpath.end()) {
			repeatNum++;
			if(makeEdges) edges.push_back(workpath[i]);	
		}
	}
	return repeatNum;
}

int findMinRepeat(vector<okPath> &workPath, vector<okPath> &backPath, vector<bestPair> &pairs)
{
	int minRep = INT_MAX;
	int repNum = 0;
	vector<int> edges;
	for(unsigned int i = 0; i < workPath.size(); i++) {
		for(unsigned int j = 0; j < backPath.size(); j++) {
			repNum = repeatEdges(workPath[i].edges, backPath[j].edges, edges, false);
			if(repNum < minRep) minRep = repNum;
			if(minRep == 0) break;
		}
		if(minRep == 0) break;
	}
	for(unsigned int i = 0; i < workPath.size(); i++) {
		for(unsigned int j = 0; j < backPath.size(); j++) {
			edges.clear();
			repNum = repeatEdges(workPath[i].edges, backPath[j].edges, edges, true);
			if(repNum == minRep) pairs.push_back(bestPair(i, j, edges));
		}
	}
	return minRep;
}

//你要完成的功能总入口
void search_route(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
{
#ifdef TIME
 	for(int i = 0; i < 10; i++) arr[i] = 0;
#endif
	//srand(9999999);
	srand((unsigned int)time(0));
	bool fmin = true; 
	bool hasPath = false; //当前迭代结束，是否还有有效路径

	vector<okPath> workPath;
	vector<okPath> backPath;
	unsigned int minWPindex = 0;
	vector<bestPair> pairs;
	unsigned int minPindex = 0;
	
	okPath wkBest;
	okPath bkBest;
	int repMin = INT_MAX;
	int costMin = INT_MAX;

	int WorB; //标志：0 寻找主路径； 1 寻找备用路径
	bool first = true; //找寻主路径
	int spNodesNum = 0;
	//SK迭代中的变量定义
	int min, addl;
	int ll, rd;
	vector<int> vecaddl;
	int addj;

	Dijkstra work;
	work.init(topo, edge_num, demand);

beginWorB:
	//cout << "begin w or b" << endl;
	if(!first) work.WorB = 1; //改变，寻找备用路径
	WorB = work.WorB;

	spNodesNum = work.includingSet[WorB].size();
	if(spNodesNum == 0) { // 某条路径必经节点为0的情况
		bitset<maxn> rmNodes;
		rmNodes.reset();
		Path tmp;
		okPath tmp2;
		work.dijkstra_2nodes(work.start, work.target, rmNodes);
		work.BuildPath(work.start, work.target, tmp);
		tmp2.cost = tmp.cost;
		tmp2.edges = tmp.edges;
		reverse(tmp2.edges.begin(), tmp2.edges.end());
		if(WorB == 0) workPath.push_back(tmp2);
		else backPath.push_back(tmp2);
	}
	//这个D只包含必经节点，起点，终点。这里面的点的下标和includingSet中的下标必须一致，起点放最后一行，终点放最后一列。
	Path D[spNodesNum + 1][spNodesNum + 1];	//i:表示起点编号 j:表示终点编号;两点之间的最短路径矩阵,
	memset(D, 0, sizeof(D)); //初始化矩阵D

	vector<vector<vector<Path>>> fl(spNodesNum, vector<vector<Path>>(spNodesNum));

	//Dijkstra构造两两之间的最短路径，若路径不存在其权重为-1
	bitset<maxn> rmNodes;
	for(int i = 0; i < spNodesNum; i++) {	//在special nodes中遍历起点
		rmNodes.reset();
		rmNodes.set(work.start);
		rmNodes.set(work.target);	
		work.dijkstra(work.includingSet[WorB][i], rmNodes);	//移除起点和终点进行计算
		for(int j = 0; j < spNodesNum; j++){
			if(i == j) continue;
			work.BuildPath(work.includingSet[WorB][i], work.includingSet[WorB][j], D[i][j]);
			D[i][j].printPath(work);
		}

		rmNodes.reset();
		rmNodes.set(work.start);	//只移除起点进行计算
		work.dijkstra(work.includingSet[WorB][i], rmNodes);

		work.BuildPath(work.includingSet[WorB][i], work.target, D[i][spNodesNum]);
		D[i][spNodesNum].printPath(work);
	}
	rmNodes.reset();
	rmNodes.set(work.target);	//只移除终点进行计算
	work.dijkstra(work.start, rmNodes);	
	for(int j = 0; j < spNodesNum; j++){
		work.BuildPath(work.start, work.includingSet[WorB][j], D[spNodesNum][j]);
		D[spNodesNum][j].printPath(work);
	}

beginSK: //SK迭代
	//cout << "begin sk" << endl;
	for(int i = 0; i < spNodesNum; i++) { //初始化fl
		for(int j = 0; j < spNodesNum; j++) {
			fl[i][j].clear();
		}
	}

	for(int i = 0; spNodesNum != 1 && i < spNodesNum; i++) {	//i:只表示需要循环的次数
		if(i == 0) {
			for(int j = 0; j < spNodesNum; j++) {	//j:遍历D的起点--vi
				for(int k = 0; k < spNodesNum; k++) {	//k:遍历D的终点--vl
					if(j == k) continue;
					if(D[j][k].cost > 0 && D[k][spNodesNum].cost > 0) {
						Path tmp = D[j][k];
						Path tmp2 = D[k][spNodesNum];
						D[j][k].handleCycle(D[k][spNodesNum], work, WorB);
						if(D[j][k].cost > 0) {
							fl[0][j].push_back(D[j][k] + D[k][spNodesNum]);
 						}
						else {
							D[j][k] = tmp;
							D[k][spNodesNum].handleCycle(D[j][k], work, WorB);
							if(D[k][spNodesNum].cost > 0) {
								fl[0][j].push_back(D[j][k] + D[k][spNodesNum]);
							}	
						}
						D[j][k] = tmp;
						D[k][spNodesNum] = tmp2;
					}
				}	
			}
		}
		else {
			for(int j = 0; j < spNodesNum; j++) {	//j:遍历D的起点
				for(int k = 0; k < spNodesNum; k++) {	//k:遍历D的终点
					if(j == k) continue;
					if(D[j][k].cost <= 0) {
						continue;
					}
					min = INT_MAX;
					addl = -1;							
					vecaddl.clear();
					for(unsigned int l = 0; l < fl[i - 1][k].size(); l++) {
						if(work.includingSet[WorB][j] == fl[i - 1][k][l].target) continue;
						vecaddl.push_back(l);
						if(D[j][k].cost + fl[i - 1][k][l].cost < min) {
							min = D[j][k].cost + fl[i - 1][k][l].cost;
							addl = l;
						}	
					}
					bool find = false;
					if(addl != -1) {	//找到一条路径（若addl==-1则之前也不可能有这条路径）
						Path tmp = D[j][k];
						if(fmin) {
							D[j][k].handleCycle(fl[i - 1][k][addl], work, WorB);
							if(D[j][k].cost != -1) {
								find = true;
								fl[i][j].push_back(D[j][k] + fl[i - 1][k][addl]);
								fl[i][j].back().i = i - 1;
								fl[i][j].back().j = k;
								fl[i][j].back().k = addl;
							}
						}
						else {
							rd = rand() % vecaddl.size(); 
							ll = vecaddl[rd];
							D[j][k].handleCycle(fl[i - 1][k][ll], work, WorB);
							if(D[j][k].cost != -1) {
								find = true;
								fl[i][j].push_back(D[j][k] + fl[i - 1][k][ll]);
								fl[i][j].back().i = i - 1;
								fl[i][j].back().j = k;
								fl[i][j].back().k = ll;
							}
						}
						D[j][k] = tmp;	
					
						//SK--(2)--(3)
						if(find == false) {
							for(unsigned int m = 0; m < fl[i - 1][j].size(); m++) {
								if(fl[i - 1][j][m].target == work.includingSet[WorB][k] &&\
								  (int)fl[i - 1][j][m].spNum >= i + 1) {
									fl[i][j].push_back(fl[i - 1][j][m]);
									break;
								}
							}
						}
						if(find && (int)fl[i][j].back().spNum >= i + 1) {
							for(unsigned int m = 0; m < fl[i - 1][j].size(); m++) {
								if(fl[i - 1][j][m].target == work.includingSet[WorB][k] &&\
								  fl[i - 1][j][m].spNum == fl[i][j].back().spNum &&\
 								  fl[i - 1][j][m].cost < fl[i][j].back().cost) {
									
									fl[i][j].back() = fl[i - 1][j][m];
									break;
								}
							}
						}
					}
					else {
						for(unsigned int m = 0; m < fl[i - 1][j].size(); m++) {
							if(fl[i - 1][j][m].target == work.includingSet[WorB][k] &&\
							  (int)fl[i - 1][j][m].spNum >= i + 1) {
								fl[i][j].push_back(fl[i - 1][j][m]);
								break;
							}
						}			
					}
				} // for k	
			} // for j
		} //if i == 0 or i != 0 
		
		hasPath = false; //判断是否存在有效路径
		for(int j = 0; j < spNodesNum; j++) {
			if(fl[i][j].size() > 0)	{
				hasPath = true;
				break;
			}	
			if(hasPath) break;
		}

		if(hasPath == true) continue;
		else break;	
	}//for i

	//vi = work.start;
	if(spNodesNum == 1) {
		hasPath = true;
		fl[0][0].push_back(D[0][1]);
		fl[0][0].back().printPath(work);
	}
	for(int i = 0; hasPath && i < spNodesNum; i++) {	//遍历D中的终点，也就是f中的起点
		min = INT_MAX;
		addj = -1;

		for(unsigned int j = 0; j < fl[spNodesNum - 1][i].size(); j++) {	//遍历f中的终点
			if(D[spNodesNum][i].cost > 0 &&\
			   D[spNodesNum][i].cost + fl[spNodesNum - 1][i][j].cost < min) {
					min = D[spNodesNum][i].cost + fl[spNodesNum - 1][i][j].cost;
					addj = j;
			}
		}

		if(addj != -1) {
			Path tmp = D[spNodesNum][i];

			D[spNodesNum][i].handleCycle(fl[spNodesNum - 1][i][addj], work, WorB);	
			if(D[spNodesNum][i].cost != -1) {
				okPath tmp;
				tmp.cost = D[spNodesNum][i].cost + fl[spNodesNum - 1][i][addj].cost;
				for(int index = D[spNodesNum][i].edges.size() - 1; index >= 0; index--)
					tmp.edges.push_back(D[spNodesNum][i].edges[index]);
				int tmpi, tmpj, tmpk;
				for(int indexi = spNodesNum - 1, indexj = i, indexk = addj; 1;) {
					for(int index = fl[indexi][indexj][indexk].edges.size() - 1; index >= 0; index--) {
						tmp.edges.push_back(fl[indexi][indexj][indexk].edges[index]);
					}
					if(fl[indexi][indexj][indexk].i == -2 || fl[indexi][indexj][indexk].i == -1) break;
					tmpi = indexi, tmpj = indexj, tmpk = indexk;
					indexi = fl[tmpi][tmpj][tmpk].i;
					indexj = fl[tmpi][tmpj][tmpk].j;
					indexk = fl[tmpi][tmpj][tmpk].k;
				}
				if(WorB == 0) workPath.push_back(tmp);
				else backPath.push_back(tmp);
			}

			D[spNodesNum][i] = tmp;
		}
	}

	if(WorB == 0) {
		if(workPath.size() == 0) {
			if(fmin) fmin = false;
			goto beginSK;
		}
		//特技：取权重最小的workPath，整条路径的权值提高。
		min = INT_MAX;
		for(unsigned int i = 0; i < workPath.size(); i++) {
			workPath[i].changeCost();
			if(workPath[i].cost < min) minWPindex = i;
		}
		for(unsigned int i = 0; i < workPath[minWPindex].edges.size(); i++) {
			allEdges[workPath[minWPindex].edges[i]].phe += 100;
		}

		first = false;
		fmin = true;
		goto beginWorB;
	}
	else {
		if(backPath.size() == 0) {
			if(fmin) fmin = false;
			goto beginSK;
		}
		for(unsigned int i = 0; i < backPath.size(); i++) backPath[i].changeCost();
	}

	pairs.clear();
	int repNum = findMinRepeat(workPath, backPath, pairs); //找到重边最少的边对
	
	min = INT_MAX;
	for(unsigned int i = 0; i < pairs.size(); i++) {
		if(workPath[pairs[i].index1].cost + backPath[pairs[i].index2].cost < min) {
			min = workPath[pairs[i].index1].cost + backPath[pairs[i].index2].cost;
			minPindex = i;
		}
	}
	if(repNum < repMin || (repNum == repMin && min < costMin)) {
		costMin = min;
		wkBest = workPath[pairs[minPindex].index1];
		bkBest = backPath[pairs[minPindex].index2];
	}

	//if(pairs[0].repEdges.size() != 0 && repNum < repMin) {
	if(0) {
		repMin = repNum;
		//cout << "repNum:" << pairs[0].repEdges.size() << endl;
		srand(9999999);
		fmin = true; 
		first = true; //找寻主路径
		work.WorB = 0;
		workPath.clear();
		backPath.clear();
		for(unsigned int i = 0; i < pairs[0].repEdges.size(); i++) {
			allEdges[pairs[0].repEdges[i]].phe += 1000;
		}
		goto beginWorB;
	}
	else {
		if(repNum < repMin) repMin = repNum;
		for(unsigned int i = 0; i < wkBest.edges.size(); i++) {
			record_result(WORK_PATH, wkBest.edges[i]);
		}
		for(unsigned int i = 0; i < bkBest.edges.size(); i++) {
			record_result(BACK_PATH, bkBest.edges[i]);
		}
	}

#ifdef DEBUG
	cout << "\nwork path----------------------------------------" << endl;
	for(unsigned int i = 0; i < workPath.size(); i++) {
		workPath[i].judgePath(work, 0);
		workPath[i].printPath();
	}
	cout << "\nback path----------------------------------------" << endl;
	for(unsigned int i = 0; i < backPath.size(); i++) {
		backPath[i].judgePath(work, 1);
		backPath[i].printPath();
	}
	cout << "\ncost1:" << wkBest.cost << endl;
	cout << "cost2:" << bkBest.cost << endl;
	cout << "cost:" << costMin << endl;
	cout << "repeatNum:" << repMin << endl;
#endif
#ifdef TIME
	double sumtime = arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6];
	cout << "= " << arr[0] << " " << arr[0] / sumtime << endl;
	cout << "+ " << arr[1] << " " << arr[1] / sumtime << endl;
	cout << "handleCycle " << arr[2] << " " << arr[2] / sumtime << endl;
	cout << "dij init " << arr[3] << " " << arr[3] / sumtime << endl;
	cout << "add edges " << arr[4] << " " << arr[4] / sumtime << endl;
	cout << "dij " << arr[5] << " " << arr[5] / sumtime << endl;
	cout << "dij2 " << arr[7] << " " << arr[7] / sumtime << endl;
	cout << "buildpath " << arr[6] << " " << arr[6] / sumtime << endl;
	cout << "sum " << sumtime << endl; 
#endif
	return;
}
