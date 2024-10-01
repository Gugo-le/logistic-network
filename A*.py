import math
import heapq

# 유클리드 거리 계산 함수 (휴리스틱)
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
graph = {
    'Warehouse': {'A': 2, 'B': 5, 'C': 1},
    'A': {'Warehouse': 2, 'C': 3, 'D': 4},
    'B': {'Warehouse': 5, 'D': 2},
    'C': {'Warehouse': 1, 'A': 3, 'D': 8},
    'D': {'A': 4, 'B': 2, 'C': 8, 'Destination': 6},
    'Destination': {'D': 6}
}

# 각 노드의 좌표 (x, y)
positions = {
    'Warehouse': (0, 0),
    'A': (2, 3),
    'B': (5, 1),
    'C': (1, 1),
    'D': (4, 5),
    'Destination': (6, 6)
}

# A*로 창고에서 목적지까지 경로 찾기
distance, path = a_star(graph, 'Warehouse', 'Destination', positions)
print(f"A* 최단 거리: {distance}, 경로: {path}")