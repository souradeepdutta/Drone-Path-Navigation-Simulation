import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt

# Step 1: Fetch Map Data
place_name = "Chennai, India"
graph = ox.graph_from_place(place_name, network_type='drive')
print("Fetched map")

# Step 2: Set Edge Weights and Define Obstacles
for u, v, data in graph.edges(data=True):
    data['weight'] = data.get('length', 1)

# Define obstacle nodes (example list, can be set as needed)
#obstacle_nodes = [list(graph.nodes())[i] for i in range(10, 2000,50)]  # Change range for different obstacles
obstacle_nodes = []

for node in obstacle_nodes:
    for neighbor in list(graph.neighbors(node)):
        if graph.has_edge(node, neighbor):
            graph[node][neighbor][0]['weight'] = float('inf')  # Set a high weight to avoid

print("Obstacles set by increasing edge weights")

# Step 3: Implement Bellman-Ford Algorithm
def bellman_ford(graph, start, goal):
    distance = {node: float('inf') for node in graph}
    distance[start] = 0
    predecessor = {node: None for node in graph}

    for _ in range(len(graph) - 1):
        for u, v, data in graph.edges(data=True):
            cost = data['weight']
            if distance[u] + cost < distance[v]:
                distance[v] = distance[u] + cost
                predecessor[v] = u

    # Check for negative-weight cycles
    for u, v, data in graph.edges(data=True):
        cost = data['weight']
        if distance[u] + cost < distance[v]:
            raise ValueError("Graph contains a negative-weight cycle")

    return distance, predecessor

# Step 4: Run Bellman-Ford for Obstacle-Aware Pathfinding
start_node = list(graph.nodes())[0]  # Example start node
goal_node = list(graph.nodes())[-1]  # Example goal node

print("Running Bellman-Ford with obstacles")

# Run Bellman-Ford
bellman_ford_distances, bellman_ford_predecessors = bellman_ford(graph, start_node, goal_node)

# Reconstruct Bellman-Ford path to the goal node
bellman_ford_path = []
if bellman_ford_distances[goal_node] < float('inf'):  # Check if the goal is reachable
    current = goal_node
    while current is not None:
        bellman_ford_path.append(current)
        current = bellman_ford_predecessors[current]
    bellman_ford_path = bellman_ford_path[::-1]
else:
    print("Goal node is not reachable using Bellman-Ford.")

# Step 5: Visualize Only the Bellman-Ford Path
fig, ax = ox.plot_graph(graph,node_size=0, show=False, close=False)

# Plot Bellman-Ford path only
if bellman_ford_path:
    ox.plot_graph_route(graph, bellman_ford_path, route_linewidth=2, route_color='red', ax=ax, orig_dest_node_color='orange', orig_dest_node_size=100)

plt.show()
