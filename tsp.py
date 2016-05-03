from re import compile
from math import sqrt
from collections import defaultdict
from sys import maxsize
from time import clock
from itertools import product, permutations

from sklearn.cluster import KMeans
from sklearn.cluster import Birch
from sklearn.cluster import DBSCAN
from sklearn.cluster import AgglomerativeClustering
from ortools.constraint_solver import pywrapcp


class DistanceMatrix(object):
  def __init__(self,matrix, points, high_level=False, gamma=None):
    self.high_level = high_level
    self.gamma = gamma if gamma else None
    self.matrix = matrix
    self.points = points
    self.num_points = len(matrix)
  def num_points(self):
    return self.num_points
  def distance(self, from_node, to_node):
    return self.matrix[from_node][to_node] if not self.high_level else self.matrix[from_node][to_node][0]

def euclidean_distance(x,y):
  return sqrt(pow(y[0] - x[0], 2) + pow(y[1] - x[1], 2))


def points_from_file(file_path):
  pattern = compile(r'\s*(\d+)\s+([0-9\.e\+]+)\s+([0-9\.e\+]+).*')
  points = []
  with open(file_path,'r') as f:
    for line in f:
      m = pattern.match(line)
      if m:
        points.append((int(m.group(1)) - 1, float(m.group(2)), float(m.group(3))))
  return points

def load_matrix(points,gamma=None):
  X = 1
  Y = 2
  gamma = gamma if gamma else None
  matrix = {}
  for a in points:
    matrix[a[0]] = {}
    for b in points:
      matrix[a[0]][b[0]] = euclidean_distance((a[X],a[Y]),(b[X],b[Y]))
  return DistanceMatrix(matrix,points,gamma=gamma)


def load_matrices_from_labels(points, labels):
  # This function takes in a list of points as imported from the input file, and
  # uses the labels to group them into matrices representing clusters. It
  # returns a tuple of the format (high-level distance matrix, dict{label:low-
  # level distance matrices}. This is the function to use AFTER clustering.
  # for correctness, the list of labels must match the list of points. It is
  # recommended that before running this function you use the following assert:
  # (assuming that points, formatted for scikit is named `data`)
  # assert(all([p[1] == d[0] and p[2] == d[1] for p,d in zip(points, data)]))
  assert(len(points) == len(labels))

  clusters = defaultdict(list)
  for point,label in zip(points,labels):
    clusters[label].append(point)

  # Convert the low-level clusters into distance matrices
  G = {}
  for label,cluster in clusters.items():
    G[label] = {}
    for i in range(len(cluster)):
      c,x,y = cluster[i]
      G[label][i] = c
      cluster[i] = (i,x,y)

  for label in clusters:
    clusters[label] = load_matrix(clusters[label],gamma=G[label])

  hl_points = list(labels)
  hl_matrix = {}
  L = set(labels)
  for label in L:
    hl_matrix[label] = {}
    for other_label in L:
      if label == other_label:
        hl_matrix[label][other_label] = (0.0, label, other_label)
      else:
        # compute minimal distance edge between the two clusters
        min_point = 0
        min_other_point = 0
        min_distance = maxsize
        for i in range(len(labels)):
          for j in range(len(labels)):
            if labels[i] == label and labels[j] == other_label:
              a,b = points[i],points[j]
              a,b = (a[1],a[2]),(b[1],b[2])
              d = euclidean_distance(a,b)
              if d < min_distance:
                min_distance = d
                min_point = points[i]
                min_other_point = points[j]
        hl_matrix[label][other_label] = (min_distance,min_point,min_other_point)
  hl_distance_matrix = DistanceMatrix(hl_matrix,hl_points,high_level=True)
  return hl_distance_matrix, clusters, G


def cp_tsp_solve(matrix, depot):
  if matrix.num_points:
    # Set a global parameter.
    param = pywrapcp.RoutingParameters()
    param.use_light_propagation = False
    pywrapcp.RoutingModel.SetGlobalParameters(param)
    routing = pywrapcp.RoutingModel(matrix.num_points, 1)

    parameters = pywrapcp.RoutingSearchParameters()
    # Setting first solution heuristic (cheapest addition).
    parameters.first_solution = 'PathCheapestArc'
    # Disabling Large Neighborhood Search, comment out to activate it.
    parameters.no_lns = True
    parameters.no_tsp = False

    matrix_callback = matrix.distance
    routing.SetArcCostEvaluatorOfAllVehicles(matrix_callback)
    routing.SetDepot(depot)

    # Solve, returns a solution if any.
    assignment = routing.SolveWithParameters(parameters, None)
    if assignment:
      route_number = 0
      node = routing.Start(route_number)
      route = []
      while not routing.IsEnd(node):
        route.append(int(node))
        node = assignment.Value(routing.NextVar(node))
      route.append(depot)
      # Return Cost, Route
      if matrix.gamma:
        for i in range(len(route)):
          route[i] = matrix.gamma[route[i]]
      return float(assignment.ObjectiveValue()), route
    else:
      print 'No solution found.'
  else:
    return 0.0,[]


def cluster_test(file_path,num_clusters):
  points = points_from_file(file_path)
  X = [[p[1],p[2]] for p in points]
  est = KMeans(n_clusters=num_clusters)
  est.fit(X)
  labels = est.labels_
  CSV = [','.join([str(points[i][0]),str(points[i][1]),str(points[i][2]),str(labels[i])]) for i in range(len(labels))]
  return CSV


def compute_depots(clusters, matrix, gamma, per_cluster=False):
    """
    This is where we actually map the high-level TSP onto the final TSP solution
    by figuring out which nodes should be used as start/endpoints of each cluster
    which in turn determines the routes.
    """
    args = [clusters[cid].points for cid in sorted(clusters.keys())]

    min_D = maxsize
    min_R = None
    for depot_set in product(*args):
        depot_set = [(i, depot_set[i]) for i in range(len(depot_set))]
        for depot_route in permutations(depot_set):

            c = 0.0
            for i in range(len(depot_route) - 1):
                gid1, point1 = depot_route[i]
                point1 = gamma[gid1][point1[0]]
                gid2, point2 = depot_route[i+1]
                point2 = gamma[gid2][point2[0]]
                c += matrix.distance(point1,point2)

            #compute_route_cost(depot_route, matrix, gamma)
            if min_D > c:
                min_D = c
                min_R = depot_route
    if not per_cluster:
        min_R = [gamma[gid][p[0]] for gid,p in min_R]
    return min_R, min_D


def clustered_tsp_solve(points, num_clusters, estimator=None, labels=None, basic=True, depots=None):
  clustering_start = clock()

  if estimator is not None and labels is None:

    X = [[p[1],p[2]] for p in points]
    estimator.fit(X)
    labels = estimator.labels_

    clustering_time = clock() - clustering_start
  elif labels is not None and estimator is None:
    labels = labels
    clustering_time = 0.0
  else:
    print("The clustering TSP solver requires either a set of labels OR a clustering algorithm, but not both or neither.")
    exit()

  hl_matrix, clusters, G = load_matrices_from_labels(points,labels)

  C = 0.0
  R = []

  ll_cluster_solve_start = clock()

  if depots is not None:
      depots = sorted(depots, key=lambda t:t[0])
      print(depots)
      for label,cluster in clusters.items():
        R.append(cp_tsp_solve(cluster,depots[label][1][0]))
        C += R[-1][0]
  else:
    for label,cluster in clusters.items():
      R.append(cp_tsp_solve(cluster,0))
      C += R[-1][0]

  ll_cluster_solve_time = clock() - ll_cluster_solve_start
  hl_cluster_solve_start = clock()

  hl_cost, hl_route = cp_tsp_solve(hl_matrix,0)

  hl_cluster_solve_time = clock() - hl_cluster_solve_start
  total_cluster_solve_time = clock() - clustering_start
  C += hl_cost

  if basic:
    return C, R, hl_route
  else:
    return C, R, hl_route, (clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time)


def cluster_tsp_vs_cp_tsp(file_path,num_clusters):
  print("SOLVING: {0} USING {1} CLUSTERS".format(file_path,num_clusters))
  optimal_start = clock()
  matrix = load_matrix(points_from_file(file_path))
  optimal_cost, route = solve_tsp(matrix,0)
  optimal_solve_time = clock() - optimal_start
  # print("\nOPTIMAL TSP: COST: {0} RUNTIME: {1}\n\tSOLUTION: {2}".format(optimal_cost, optimal_solve_time, route))
  # print("\nOPTIMAL TSP: COST: {0} RUNTIME: {1}".format(optimal_cost, optimal_solve_time))

  points = points_from_file(file_path)
  cluster_start = clock()

  # Do Clustering Here to Generate a set of Labels:
  # K-Means Clustering Example
  X = [[p[1],p[2]] for p in points]

  clustering_start = clock()
  est = KMeans(n_clusters=num_clusters)
  est.fit(X)
  clustering_time = clock() - clustering_start
  labels = est.labels_

  C = 0.0
  cluster_solve_start = clock()
  hl_matrix, clusters = load_matrices_from_labels(points,labels)
  for label,cluster in clusters.items():
    start = clock()
    cost, route = solve_tsp(cluster,0)
    end = clock()
    C += cost
    # print("\nLOW LEVEL SOLUTIONS: CLUSTER: {0} COST: {1} RUNTIME: {2}\n\tSOLUTION: {3}".format(label, cost, (end - start), route))
    # print("\nLOW LEVEL SOLUTIONS: CLUSTER: {0} COST: {1} RUNTIME: {2}".format(label, cost, (end - start)))

  # print(hl_matrix)
  high_level_start = clock()
  cost, route = solve_tsp(hl_matrix,0)

  high_level_cluster_solution_time = clock() - high_level_start
  cluster_solve_time = clock() - cluster_solve_start
  cluster_total_time = clock() - clustering_start

  C += cost
  # print("\nHIGH LEVEL SOLUTION: COST: {0} RUNTIME: {1}\n\tSOLUTION: {2}".format(cost, (end - start), route))
  # print("\nHIGH LEVEL SOLUTION: COST: {0} RUNTIME: {1}".format(cost, (end - start)))
  # print("\nTOTAL CLUSTER SOLUTION COST: {0} RUNTIME: {1} ~> {2}% OPTIMAL".format(C, cluster_total_time, optimal_cost / C * 100.0))
  # num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality, speedup
  return [matrix.num_points, optimal_cost, optimal_solve_time, num_clusters, clustering_time, high_level_cluster_solution_time, cluster_solve_time, cluster_total_time, C, optimal_cost / C, optimal_solve_time / cluster_total_time]


def test_depot_calculation():
    points = points_from_file('tsps/berlin52.txt')
    matrix = load_matrix(points)
    X = [[p[1],p[2]] for p in points]
    est = KMeans(n_clusters=2)
    est.fit(X)
    labels = est.labels_
    hl_matrix, clusters, G = load_matrices_from_labels(points,labels)
    compute_depots(clusters, matrix, G, per_cluster=True)


def test_birch_with_depot_calculation():
    points = points_from_file('tsps/berlin52.txt')
    matrix = load_matrix(points)
    X = [[p[1],p[2]] for p in points]
    est = Birch(n_clusters=3)
    est.fit(X)
    labels = est.labels_
    hl_matrix, clusters, G = load_matrices_from_labels(points,labels)
    depots, C = compute_depots(clusters, matrix, G, per_cluster=True)
    depots_actual, _ = compute_depots(clusters, matrix, G)
    cluster_optimal_cost, R, hl_route = clustered_tsp_solve(points, 3, labels=labels, depots=depots)
    cluster_optimal_cost += C

    print(depots_actual)
    print(R,C)

    for depot in depots_actual:
        for r in R:
            if r[1][0] == depot:
                for point in r[1]:
                    print(matrix.points[point])
        print('')

def test_kmeans_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):
        # KMeans clustering
        estimator = KMeans(n_clusters=num_clusters)

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('kmeans_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def test_birch_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):
        # Birch clustering
        estimator = Birch(n_clusters=num_clusters)

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('birch_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def test_dbscan_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):
        # DBSCAN clustering
        estimator = DBSCAN(eps=0.3, min_samples=10)

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('dbscan_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def test_agglomerative_ward_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):

        # DBSCAN clustering
        # estimator = DBSCAN(eps=0.3, min_samples=10)

        # Agglomerative
        estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='ward')

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('agglom_ward_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def test_agglomerative_complete_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):
        # Agglomerative
        estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='complete')

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('agglom_complete_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def test_agglomerative_ave_clustering():
    Results = {}
    files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
             'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
             'tsps/d1291.txt','tsps/d1655.txt',]
    for file_path in files:
      points = points_from_file(file_path)

      optimal_start = clock()
      optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
      optimal_solve_time = clock() - optimal_start

      for num_clusters in range(2,21):
        # Agglomerative
        estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='average')

        cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
        clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
        Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                     ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                     optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
        print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
    print("Writing Output to CSV")
    CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
    for file_path in files:
      for num_clusters in range(2,21):
        CSV.append(Results[(file_path,num_clusters)])
    with open('agglom_ave_results.csv','w+') as csv_file:
      csv_file.write('\n'.join(CSV))


def main():
  Results = {}
  files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
           'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
           'tsps/d1291.txt','tsps/d1655.txt',]
  for file_path in files:
    points = points_from_file(file_path)

    optimal_start = clock()
    optimal_cost, _ = cp_tsp_solve(load_matrix(points), 0)
    optimal_solve_time = clock() - optimal_start

    for num_clusters in range(2,21):

      # DBSCAN clustering
      # estimator = DBSCAN(eps=0.3, min_samples=10)

      # Agglomerative
      #estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='ward')
      #estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='complete')
      estimator = AgglomerativeClustering(n_clusters=num_clusters, affinity='euclidean', linkage='average')

      cluster_optimal_cost, R, hl_route, times = clustered_tsp_solve(points, num_clusters, estimator, basic=False)
      clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time = times
      Results[(file_path,num_clusters)] = ','.join(str(e) for e in [len(points), optimal_cost, optimal_solve_time, num_clusters, clustering_time, hl_cluster_solve_time,
                                                                   ll_cluster_solve_time, total_cluster_solve_time, cluster_optimal_cost,
                                                                   optimal_cost / cluster_optimal_cost, optimal_solve_time / total_cluster_solve_time])
      print("CLUSTERING TSP SOLUTION({0},{7}) COST: {1}\n\tHIGH-LEVEL ROUTE: {2}\n\tTIME SPENT CLUSTERING: {3}\n\tTIME SPENT SOLVING LOW-LEVEL TSPS: {4}\n\tTIME SPENT SOLVING HIGH-LEVEL TSP: {5}\n\tTIME SPENT TOTAL: {6}\n".format(file_path, cluster_optimal_cost, hl_route, clustering_time, ll_cluster_solve_time, hl_cluster_solve_time, total_cluster_solve_time, num_clusters))
  print("Writing Output to CSV")
  CSV = ['num_cities,optimal_cost,optimal_solve_time,num_clusters,clustering_time,high_level_cluster_solution_time,cluster_solve_time,cluster_total_time,cluster_optimal_cost,cluser_optimality,speedup']
  for file_path in files:
    for num_clusters in range(2,21):
      CSV.append(Results[(file_path,num_clusters)])
  with open('agglom_ave_results.csv','w+') as csv_file:
    csv_file.write('\n'.join(CSV))


  # files = ['tsps/berlin52.txt','tsps/bier127.txt','tsps/a280.txt','tsps/d493.txt',
  #          'tsps/rat575.txt','tsps/d657.txt','tsps/u724.txt','tsps/vm1084.txt',
  #          'tsps/d1291.txt','tsps/d1655.txt',]
  # for file_path in files:
  #   for num_clusters in range(2,21):
  #     CSV.append(','.join([str(element) for element in cluster_tsp_vs_cp_tsp(file_path,num_clusters)]))
  # with open('k_means_results.csv','w+') as csv_file:
  #   csv_file.write('\n'.join(CSV))

  # CSV = ['point,X,Y,label']
  # for file_path in files:
  #   for num_clusters in [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]:
  #     csv_file = '\n'.join(CSV + cluster_test(file_path,num_clusters))
  #     with open('clusters/k_means_{0}_{1}.csv'.format(file_path[5:-4],num_clusters),'w+') as f:
  #       f.write(csv_file)

if __name__ == '__main__':
  # main()
  # test_depot_calculation()
  test_birch_with_depot_calculation()
