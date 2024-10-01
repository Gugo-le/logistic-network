import heapq

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

# 그래프 정의 (노드: 창고, 배송지)
graph = {
    'Warehouse': {'A': 2, 'B': 5, 'C': 1},
    'A': {'Warehouse': 2, 'C': 3, 'D': 4},
    'B': {'Warehouse': 5, 'D': 2},
    'C': {'Warehouse': 1, 'A': 3, 'D': 8},
    'D': {'A': 4, 'B': 2, 'C': 8, 'Destination': 6},
    'Destination': {'D': 6}
}

# 창고에서 목적지까지 최단 경로 찾기
distance, path = dijkstra(graph, 'Warehouse', 'Destination')
print(f"최단 거리: {distance}, 경로: {path}")