import random
import matplotlib.pyplot as plt
import networkx as nx

# Create random graph
def create_random_graph(node_count, edge_count):
    graph = {i: [] for i in range(node_count)}
    existing_edges = set()

    while len(existing_edges) < edge_count:
        u = random.randint(0, node_count - 1)
        v = random.randint(0, node_count - 1)
        if u != v and (u, v) not in existing_edges and (v, u) not in existing_edges:
            weight = random.randint(1, 20)
            graph[u].append((v, weight))
            graph[v].append((u, weight))  
            existing_edges.add((u, v))

    return graph

# Dijkstra algorithm
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    visited = set()

    while len(visited) < len(graph):
        min_node = None
        min_distance = float('inf')
        for node in graph:
            if node not in visited and distances[node] < min_distance:
                min_distance = distances[node]
                min_node = node

        if min_node is None:
            break

        visited.add(min_node)

        for neighbor, weight in graph[min_node]:
            if distances[neighbor] > distances[min_node] + weight:
                distances[neighbor] = distances[min_node] + weight

    return distances

# Visualization function
def visualize(graph, distances):
    G = nx.Graph()

    for u in graph:
        for v, w in graph[u]:
            G.add_edge(u, v, weight=w)

    pos = nx.spring_layout(G, seed=42)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    node_labels = {node: f"{node}\n{distances[node]}" for node in distances if distances[node] != float('inf')}

    plt.figure(figsize=(12, 10))
    nx.draw_networkx_nodes(G, pos, node_size=30)
    nx.draw_networkx_edges(G, pos, alpha=0.3)
    nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=6)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=5)

    plt.title("Dijkstra Results (Node / Distance)")
    plt.axis('off')
    plt.show()

# MAIN
graph = create_random_graph(1000, 5000)
print("Graph created successfully. First 5 nodes:")
for i in range(5):
    print(f"{i} -> {graph[i]}")

print("\nRunning Dijkstra algorithm...")
results = dijkstra(graph, 0)

print("Shortest distances to first 10 nodes:")
for i in range(10):
    print(f"0 -> {i}: {results[i]}")

# Visualize first 30 nodes
sub_graph = {i: graph[i] for i in range(30)}
sub_distances = {i: results[i] for i in range(30)}
visualize(sub_graph, sub_distances)
