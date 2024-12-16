from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Define the distance matrix between cities
distance_matrix = [
    # Brisbane, Gold Coast, Tweed Heads, Logan, Ipswich
    [0, 79, 105, 27, 40],  # Brisbane
    [79, 0, 28, 52, 96],   # Gold Coast
    [105, 28, 0, 76, 120], # Tweed Heads
    [27, 52, 76, 0, 65],   # Logan
    [40, 96, 120, 65, 0]   # Ipswich
]

# City names for reference
city_names = ["Brisbane", "Gold Coast", "Tweed Heads", "Logan", "Ipswich"]

# Define time windows for each city (in minutes)
time_windows = [
    (0, 120),  # Brisbane
    (60, 180), # Gold Coast
    (120, 240),# Tweed Heads
    (0, 120),  # Logan
    (60, 180)  # Ipswich
]

# Function to print the solution
def print_solution(manager, routing, solution):
    print("Optimal Route:")
    route_distance = 0
    index = routing.Start(0)  # Start node
    plan_output = ""
    while not routing.IsEnd(index):
        plan_output += f"{city_names[manager.IndexToNode(index)]} -> "
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += city_names[manager.IndexToNode(index)]  # Add the end node
    print(plan_output)
    print(f"Total Distance: {route_distance} km")

# Create the routing index manager
manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)

# Create the routing model
routing = pywrapcp.RoutingModel(manager)

# Define cost of each arc
def distance_callback(from_index, to_index):
    # Convert from routing variable Index to distance matrix NodeIndex
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add time window constraints
time = 'Time'
routing.AddDimension(
    transit_callback_index,
    5,  # allow waiting time
    30,  # maximum time per vehicle
    False,  # Don't force start cumul to zero.
    time)
time_dimension = routing.GetDimensionOrDie(time)

for location_idx, (start, end) in enumerate(time_windows):
    index = manager.NodeToIndex(location_idx)
    time_dimension.CumulVar(index).SetRange(start, end)

# Solve the problem
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

solution = routing.SolveWithParameters(search_parameters)

# Print the solution
if solution:
    print_solution(manager, routing, solution)
else:
    print("No solution found!")