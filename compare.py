import heapq
import math
import time

# 다익스트라 알고리즘
def dijkstra(graph, start, target):
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    shortest_path = {}

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                shortest_path[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))
    
    # 목표 노드까지의 경로 추적
    path = []
    node = target
    while node in shortest_path:
        path.insert(0, node)
        node = shortest_path[node]
    if path:
        path.insert(0, start)
    
    return distances[target], path

# 유클리드 거리 계산 함수 (A* 휴리스틱)
def euclidean_heuristic(node, target, positions):
    x1, y1 = positions[node]
    x2, y2 = positions[target]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# A* 알고리즘
def a_star(graph, start, target, positions):
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = euclidean_heuristic(start, target, positions)
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == target:
            path = []
            while current in came_from:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return g_score[target], path

        for neighbor, weight in graph[current].items():
            tentative_g_score = g_score[current] + weight

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euclidean_heuristic(neighbor, target, positions)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return float('inf'), []

# 복잡한 그래프 정의
graph = {
    'Warehouse1': {'A': 2, 'B': 5, 'C': 1},
    'Warehouse2': {'B': 3, 'E': 2},
    'A': {'Warehouse1': 2, 'C': 3, 'D': 4, 'E': 6},
    'B': {'Warehouse1': 5, 'Warehouse2': 3, 'D': 2, 'F': 4},
    'C': {'Warehouse1': 1, 'A': 3, 'D': 8, 'G': 10},
    'D': {'A': 4, 'B': 2, 'C': 8, 'Destination1': 6, 'Destination2': 8},
    'E': {'Warehouse2': 2, 'A': 6, 'F': 3, 'Destination1': 5},
    'F': {'B': 4, 'E': 3, 'G': 7, 'Destination2': 9},
    'G': {'C': 10, 'F': 7, 'Destination2': 3},
    'Destination1': {'D': 6, 'E': 5},
    'Destination2': {'D': 8, 'F': 9, 'G': 3}
}

# 각 노드의 좌표 (A*에서 사용)
positions = {
    'Warehouse1': (0, 0),
    'Warehouse2': (5, 0),
    'A': (2, 3),
    'B': (5, 1),
    'C': (1, 1),
    'D': (4, 5),
    'E': (6, 2),
    'F': (7, 4),
    'G': (9, 1),
    'Destination1': (3, 6),
    'Destination2': (8, 6)
}

# 다익스트라 알고리즘을 이용한 경로 탐색 및 시간 측정
start_time_dijkstra = time.time()
distance_dijkstra, path_dijkstra = dijkstra(graph, 'Warehouse1', 'Destination2')
end_time_dijkstra = time.time()
dijkstra_time = end_time_dijkstra - start_time_dijkstra

print(f"다익스트라 최단 거리: {distance_dijkstra}, 경로: {path_dijkstra}, 실행 시간: {dijkstra_time:.6f}초")

# A* 알고리즘을 이용한 경로 탐색 및 시간 측정
start_time_astar = time.time()
distance_astar, path_astar = a_star(graph, 'Warehouse1', 'Destination2', positions)
end_time_astar = time.time()
astar_time = end_time_astar - start_time_astar

print(f"A* 최단 거리: {distance_astar}, 경로: {path_astar}, 실행 시간: {astar_time:.6f}초")