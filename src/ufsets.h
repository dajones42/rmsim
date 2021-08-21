//	implements disjoint sets union/find with path compression and
//	union by rank
/*
Copyright © 2021 Doug Jones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

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
