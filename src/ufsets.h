//	implements disjoint sets union/find with path compression and
//	union by rank

#ifndef UFSETS_H
#define UFSETS_H

class UFSets {
	int* parent;
	int* rank;
 public:
	UFSets(int n) {
		parent= new int[n];
		rank= new int[n];
		for (int i=0; i<n; i++) {
			parent[i]= i;
			rank[i]= 0;
		}
	};
	~UFSets() {
		delete[] parent;
		delete[] rank;
	};
	int find(int x) {
		int s= x;
		while (s != parent[s])
			s= parent[s];
		while (x != s) {
			int t= parent[x];
			parent[x]= s;
			x= t;
		}
		return s;
	};
	int link(int x, int y) {
		if (rank[x] < rank[y]) {
			parent[x]= y;
			return y;
		}
		if (rank[y] < rank[x]) {
			parent[y]= x;
			return x;
		}
		rank[y]++;
		parent[x]= y;
		return y;
	};
};

#endif
