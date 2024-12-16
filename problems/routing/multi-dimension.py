from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Distance matrix between locations
distance_matrix = [
    [0, 10, 20, 30, 40],  # Technician start
    [10, 0, 15, 25, 35],  # Task 1
    [20, 15, 0, 10, 20],  # Task 2
    [30, 25, 10, 0, 15],  # Task 3
    [40, 35, 20, 15, 0],  # Task 4
]

# Priorities of tasks (higher is better)
task_priorities = [0, 10, 20, 30, 40]  # Index 0 is the technician's starting location.

# Time windows for tasks (start, end in minutes)
time_windows = [
    (0, 300),  # Technician start
    (30, 120), # Task 1
    (60, 150), # Task 2
    (90, 180), # Task 3
    (120, 240),# Task 4
]

# Number of technicians
num_technicians = 2

# Create routing index manager
manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_technicians, 0)

# Create routing model
routing = pywrapcp.RoutingModel(manager)

# Define cost of travel
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add time window constraints
time = "Time"
routing.AddDimension(
    transit_callback_index,
    30,  # Slack time allowed
    300, # Maximum time per technician
    False,  # Don't force starting cumul to zero
    time
)
time_dimension = routing.GetDimensionOrDie(time)

# Apply time windows
for location_idx, (start, end) in enumerate(time_windows):
    index = manager.NodeToIndex(location_idx)
    time_dimension.CumulVar(index).SetRange(start, end)

# Add priority dimension
priority = "Priority"
priority_callback_index = routing.RegisterUnaryTransitCallback(lambda index: task_priorities[manager.IndexToNode(index)])
routing.AddDimensionWithVehicleCapacity(
    priority_callback_index,
    0,  # No slack
    [100] * num_technicians,  # Max priority per technician
    True,  # Start cumul to zero
    priority
)
priority_dimension = routing.GetDimensionOrDie(priority)

# Solve the problem
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

solution = routing.SolveWithParameters(search_parameters)

# Print solution
def print_solution():
    total_distance = 0
    total_priority = 0
    for technician_id in range(num_technicians):
        index = routing.Start(technician_id)
        plan_output = f"Route for technician {technician_id}:\n"
        route_distance = 0
        route_priority = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            plan_output += f" {node_index} ->"
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, technician_id)
            route_priority += task_priorities[node_index]
        plan_output += " End\n"
        plan_output += f"Distance of route: {route_distance} km\n"
        plan_output += f"Priority of route: {route_priority}\n"
        print(plan_output)
        total_distance += route_distance
        total_priority += route_priority
    print(f"Total distance of all routes: {total_distance} km")
    print(f"Total priority of all routes: {total_priority}")

if solution:
    print_solution()
else:
    print("No solution found!")