SRC = 0 # source
CST = 1 # destination
DST = 2 # cost

def all_paths(edges, depot, max_cost=None):
	V = {}
	E = []
	solutions = []
	for e in edges:
		if e[SRC] not in V: V[e[SRC]] = []
		if e[DST] not in V: V[e[DST]] = []
		V[e[SRC]].append(e)
		E.append(e)
		V[e[DST]].append((e[DST],e[CST],e[SRC]))
		E.append((e[DST],e[CST],e[SRC]))
	S = [(depot,[],[depot],0)] # current node, path, visited nodes, cost
	while S:
		v,path,visited,cost = S.pop()
		if len(set(visited)) == len(V) and visited[-1] is depot:
			solutions.append((path,visited,cost))
		elif max_cost == None or cost < max_cost:
			for e in V[v]:
				if e not in path and cost + e[CST] <= max_cost:
					S.append((e[DST],
						     list(path) + [e],
						     list(visited) + [e[DST]],
						     cost + e[CST]))
	solutions.sort(key=lambda t:t[2])
	return solutions, V, E

def solutions_to_csv_str(S,E):
	csv = [','.join([e[SRC] + '-' + e[DST] for e in E]) + ',COST']
	e2i = {}
	for i in range(len(E)):
		e2i[E[i]] = i
	for P,V,c in S:
		s = [0] * len(E)
		for p in P:
			s[e2i[p]] = 1
		s = ','.join([str(i) for i in s]) + ',{0}'.format(c)
		csv.append(s)
	csv = '\n'.join(csv)
	return csv

def test_all_paths():
	a,b,c,d = 'a','b','c','d'
	E = [(a,50,c),(a,10,b),(a,45,d),(b,25,c),(b,25,d),(c,40,d)]
	S,V,E = all_paths(E,a,200)
	with open('output.csv','w+') as f:
		f.write(solutions_to_csv_str(S,E))

if __name__ == "__main__":
	test_all_paths()