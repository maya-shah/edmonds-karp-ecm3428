from typing import Tuple


def EdmondsKarp(adjacency_matrix, capacity_matrix, source, sink, graph_size) -> Tuple[int, list[list[int]]]:
    """
    Edmonds-Karp Algorithm

    :param adjacency_matrix: The adjacency matrix for the graph - neighbours of each node
    :param capacity_matrix: The capacity matrix for the graph - amount of capacity for each node
    :param source: The starting point of the graph
    :param sink: The ending point of the graph
    :param graph_size: The size of the graph - amount of nodes in the graph
    :return: The maximum flow for the graph and the residual_capacity_array for the amount of capacity left in the graph
    """
    flow = 0
    residual_capacity_array = [[0 for _ in range(graph_size)] for _ in range(graph_size)]

    while True:
        augmenting_path_capacity, augmenting_path = BFS(capacity_matrix, adjacency_matrix, residual_capacity_array,
                                                        source, sink, graph_size)

        if augmenting_path_capacity == 0:
            print(
                f"\nAugmenting path capacity: {augmenting_path_capacity}"
                f"\nThere are no more augmenting paths with free capacity.")
            break

        flow += augmenting_path_capacity

        v = sink

        path = []

        while v != source:
            u = augmenting_path[v]
            path.append(u)
            # reduce the residual capacity of the augmenting path
            residual_capacity_array[u][v] += augmenting_path_capacity
            # increase the residual capacity of the reverse edges
            residual_capacity_array[v][u] -= augmenting_path_capacity
            v = u

        path.insert(0, sink)
        path.reverse()

        print(f"\nAugmenting path capacity: {augmenting_path_capacity}\nAugmenting path: {path}\nMax flow: {flow}")

    return flow, residual_capacity_array


def BFS(capacity_matrix, adjacency_matrix, residual_capacity_array, source, sink, graph_size) -> Tuple[int, list[int]]:
    """
    Breadth-First Search

    :param capacity_matrix: The capacity matrix for the graph - amount of capacity for each node
    :param adjacency_matrix: The adjacency matrix for the graph - neighbours of each node
    :param residual_capacity_array: The residual capacity array for the graph - amount of residual capacity for each node
    :param source: The starting point of the graph
    :param sink: The ending point of the graph
    :param graph_size: The size of the graph - amount of nodes in the graph
    :return: The flow of the augmenting path and the augmenting path array
    """
    queue = [source]

    visited = [False for _ in range(0, graph_size)]
    visited[source] = True

    augmenting_paths = [-1 for _ in range(0, graph_size)]
    augmenting_paths[source] = -2

    path_capacity = [0 for _ in range(graph_size)]
    path_capacity[source] = float('inf')

    while queue:
        u = queue.pop(0)

        for v in adjacency_matrix[u]:
            if capacity_matrix[u][v] - residual_capacity_array[u][v] > 0 and augmenting_paths[v] == -1:
                augmenting_paths[v] = u
                visited[u] = True
                # get the minimum capacity for this path
                path_capacity[v] = min(path_capacity[u], capacity_matrix[u][v] - residual_capacity_array[u][v])

                if v != sink:
                    queue.append(v)
                else:
                    return path_capacity[v], augmenting_paths

    return 0, augmenting_paths


def get_adjacency_matrix(capacity_matrix) -> list[list[int]]:
    """
    Helper function to get the adjacency for the graph
    
    :param capacity_matrix: The capacity matrix for the graph - amount of capacity for each node
    :return: The adjacency matrix
    """
    adjacency_matrix = [[] for _ in range(0, len(capacity_matrix))]

    for node in range(0, len(capacity_matrix)):
        for neighbour in range(0, len(capacity_matrix[node])):
            if capacity_matrix[node][neighbour] == 0:
                continue
            else:
                adjacency_matrix[node].append(neighbour)
                adjacency_matrix[neighbour].append(node)

    return adjacency_matrix


if __name__ == '__main__':
    capacity_matrix = [[0, 3, 0, 3, 0, 0, 0],
                       [0, 0, 4, 0, 0, 0, 0],
                       [3, 0, 0, 1, 2, 0, 0],
                       [0, 0, 0, 0, 2, 6, 0],
                       [0, 1, 0, 0, 0, 0, 1],
                       [0, 0, 0, 0, 0, 0, 9],
                       [0, 0, 0, 0, 0, 0, 0]]

    adjacency_matrix = get_adjacency_matrix(capacity_matrix)

    graph_size = len(capacity_matrix)

    source, sink = 0, graph_size - 1

    print("Initial graph (capacity matrix):", *capacity_matrix, sep='\n')

    flow, residual_capacity_matrix = EdmondsKarp(adjacency_matrix, capacity_matrix, source, sink, graph_size)

    print("\nResidual capacity matrix:", *residual_capacity_matrix, sep='\n')

    print(f"\nFinal max flow: {flow}")
