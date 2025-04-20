import heapq
import time
import math

# Representasi graf kota dalam bentuk adjacency list
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'D': 2, 'E': 5},
    'C': {'A': 4, 'F': 3},
    'D': {'B': 2},
    'E': {'B': 5, 'F': 1},
    'F': {'C': 3, 'E': 1, 'G': 2},
    'G': {'F': 2}
}

# Heuristik jarak langsung ke goal 'G'
heuristic = {
    'A': 7,
    'B': 6,
    'C': 5,
    'D': 4,
    'E': 2,
    'F': 1,
    'G': 0
}

# GBFS (Greedy Best First Search)
def gbfs(start, goal):
    visited = set()
    pq = [(heuristic[start], start, [start])]
    nodes_visited = 0
    start_time = time.time()

    while pq:
        _, current, path = heapq.heappop(pq)
        nodes_visited += 1
        if current == goal:
            end_time = time.time()
            return path, nodes_visited, (end_time - start_time) * 1000
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                heapq.heappush(pq, (heuristic[neighbor], neighbor, path + [neighbor]))

# A* Search
def astar(start, goal):
    visited = set()
    pq = [(heuristic[start], 0, start, [start])]
    nodes_visited = 0
    start_time = time.time()

    while pq:
        est_total, cost_so_far, current, path = heapq.heappop(pq)
        nodes_visited += 1
        if current == goal:
            end_time = time.time()
            return path, nodes_visited, (end_time - start_time) * 1000
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                cost = cost_so_far + graph[current][neighbor]
                est = cost + heuristic[neighbor]
                heapq.heappush(pq, (est, cost, neighbor, path + [neighbor]))

# Eksekusi dan perbandingan
start, goal = 'A', 'G'

path_gbfs, nodes_gbfs, time_gbfs = gbfs(start, goal)
path_astar, nodes_astar, time_astar = astar(start, goal)

print("[Assignment 1] City Navigation")
print("GBFS:")
print("Jalur:", path_gbfs)
print("Node dikunjungi:", nodes_gbfs)
print("Waktu (ms):", f"{time_gbfs:.3f}")

print("\nA*:") 
print("Jalur:", path_astar)
print("Node dikunjungi:", nodes_astar)
print("Waktu (ms):", f"{time_astar:.3f}")
