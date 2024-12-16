import osmnx as ox
import networkx as nx
import heapq
import matplotlib.pyplot as plt

# Step 1: Fetch Map Data
place_name = "Manhattan, New York, USA"
graph = ox.graph_from_place(place_name, network_type='drive')

# Step 2: Set Edge Weights
# Use the length of the edges as weights for pathfinding
for u, v, data in graph.edges(data=True):
    data['weight'] = data.get('length', 1)

# Step 3: Define Heuristic Function for A*
def heuristic(u, v):
    # Calculate the Euclidean distance between two nodes
    return ox.distance.euclidean_dist_vec(graph.nodes[u]['y'], graph.nodes[u]['x'],
                                          graph.nodes[v]['y'], graph.nodes[v]['x'])

# Step 4: Define A* Algorithm with Obstacle Avoidance
def a_star_with_obstacles(graph, start, goal, heuristic, obstacle_nodes):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)
    
    penalty_weight = 1e6  # High penalty to avoid obstacle nodes

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in graph.neighbors(current):
            # Adjust cost if neighbor is an obstacle
            base_cost = graph[current][neighbor][0]['weight']
            if neighbor in obstacle_nodes:
                cost = base_cost + penalty_weight  # Apply a penalty to avoid obstacle
            else:
                cost = base_cost
            
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Helper function to reconstruct the path from start to goal
def reconstruct_path(came_from, current):
    # Reconstruct the path from start to goal
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]

# Step 5: Define Example Obstacle Nodes
# Define a set of obstacle nodes (these should be specific node IDs in the graph)
# For this example, replace `node_id1`, `node_id2`, etc., with actual node IDs from the graph.
#obstacle_nodes = {list(graph.nodes())[50], list(graph.nodes())[60], list(graph.nodes())[70]}
#obstacle_nodes = [list(graph.nodes())[i] for i in range(10, 6000,5)] #obstacle nodes
obstacle_nodes = []

# Step 6: Run A* with Obstacles
start_node = list(graph.nodes())[0]  # Example start node
goal_node = list(graph.nodes())[-1]    # Example goal node

print("Total no of nodes: ",len(list(graph.nodes())))

# Run A* search with obstacle avoidance
a_star_path = a_star_with_obstacles(graph, start_node, goal_node, heuristic, obstacle_nodes)
print("A* Path avoiding obstacles:", a_star_path)


# Step 7: Plot the path on the map
fig, ax = ox.plot_graph_route(graph, a_star_path, route_linewidth=2, node_size=0, bgcolor='k')
plt.show()
