import heapq
import math
import time
from itertools import permutations

# 유클리드 거리 계산 함수
def euclidean_distance(node1, node2, positions):
    x1, y1 = positions[node1]
    x2, y2 = positions[node2]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# 다익스트라 알고리즘 (한 쌍의 노드 간 최단 경로)
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_distance > distances[current_node]:
            continue
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return distances

# 다익스트라 기반 TSP: 모든 노드를 순회
def dijkstra_tsp(graph, start, positions):
    nodes = list(graph.keys())
    nodes.remove(start)
    
    shortest_distance = float('inf')
    best_path = []

    for perm in permutations(nodes):
        current_distance = 0
        current_node = start
        
        # 순열 경로에 따라 다익스트라로 경로 계산
        for next_node in perm:
            distances = dijkstra(graph, current_node)
            current_distance += distances[next_node]
            current_node = next_node
        
        distances = dijkstra(graph, current_node)
        current_distance += distances[start]
        
        if current_distance < shortest_distance:
            shortest_distance = current_distance
            best_path = [start] + list(perm) + [start]
    
    return shortest_distance, best_path

# A* 알고리즘 (한 쌍의 노드 간 경로 탐색)
def a_star(graph, start, goal, positions):
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            return g_score[current]
        
        for neighbor, weight in graph[current].items():
            tentative_g_score = g_score[current] + weight
            if tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + euclidean_distance(neighbor, goal, positions)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return float('inf')

# A* 기반 TSP: 모든 노드를 순회
def a_star_tsp(graph, start, positions):
    nodes = list(graph.keys())
    nodes.remove(start)
    
    shortest_distance = float('inf')
    best_path = []


    for perm in permutations(nodes):
        current_distance = 0
        current_node = start

        # 순열 경로에 따라 A*로 경로 계산
        for next_node in perm:
            current_distance += a_star(graph, current_node, next_node, positions)
            current_node = next_node

        # 마지막 노드에서 다시 시작점으로 돌아오는 거리 추가
        current_distance += a_star(graph, current_node, start, positions)

        # 최단 경로 및 거리 갱신
        if current_distance < shortest_distance:
            shortest_distance = current_distance
            best_path = [start] + list(perm) + [start]

    return shortest_distance, best_path


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

# 각 노드의 좌표 (이전과 동일)
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

# 다익스트라 기반 TSP 실행 및 시간 측정
start_time_dijkstra = time.time()
distance_dijkstra, path_dijkstra = dijkstra_tsp(graph, 'Warehouse1', positions)
end_time_dijkstra = time.time()
dijkstra_time = end_time_dijkstra - start_time_dijkstra

print(f"다익스트라 기반 TSP 최단 거리: {distance_dijkstra}, 경로: {path_dijkstra}, 실행 시간: {dijkstra_time:.6f}초")

# A* 기반 TSP 실행 및 시간 측정
start_time_astar = time.time()
distance_astar, path_astar = a_star_tsp(graph, 'Warehouse1', positions)
end_time_astar = time.time()
astar_time = end_time_astar - start_time_astar

print(f"A* 기반 TSP 최단 거리: {distance_astar}, 경로: {path_astar}, 실행 시간: {astar_time:.6f}초")