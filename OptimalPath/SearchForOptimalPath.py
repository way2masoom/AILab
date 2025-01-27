import heapq
from collections import deque, defaultdict

def uniform_cost_search(graph, start, goal):
    # Priority queue to store (cost, node, path)
    pq = [(0, start, [start])]
    visited = set()
    
    while pq:
        cost, node, path = heapq.heappop(pq)
        
        if node in visited:
            continue
        visited.add(node)
        
        # Goal reached
        if node == goal:
            return cost, path
        
        # Add neighbors to the priority queue
        for neighbor, weight in graph[node]:
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path + [neighbor]))
    
    return float('inf'), []  # If no path exists

# BFS for unweighted graphs
def bfs_unweighted(graph, start, goal):
    queue = deque([(start, [start])])
    visited = set()
    
    while queue:
        node, path = queue.popleft()
        
        if node in visited:
            continue
        visited.add(node)
        
        if node == goal:
            return path
        
        for neighbor, _ in graph[node]:
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
    
    return []  # If no path exists

# Example usage
if __name__ == "__main__":
    # Weighted graph represented as an adjacency list
    weighted_graph = {
        'A': [('B', 1), ('C', 4)],
        'B': [('A', 1), ('C', 2), ('D', 5)],
        'C': [('A', 4), ('B', 2), ('D', 1)],
        'D': [('B', 5), ('C', 1)]
    }

    # Unweighted graph derived from the above by ignoring weights
    unweighted_graph = {k: [(neighbor, 1) for neighbor, _ in v] for k, v in weighted_graph.items()}

    start_node = 'A'
    goal_node = 'D'

    # Uniform Cost Search
    ucs_cost, ucs_path = uniform_cost_search(weighted_graph, start_node, goal_node)
    print(f"Uniform Cost Search: Cost = {ucs_cost}, Path = {ucs_path}")

    # BFS for unweighted graphs
    bfs_path = bfs_unweighted(unweighted_graph, start_node, goal_node)
    print(f"BFS (Unweighted): Path = {bfs_path}")
