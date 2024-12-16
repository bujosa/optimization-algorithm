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


# Output:
# Optimal Route:
# Brisbane -> Logan -> Gold Coast -> Tweed Heads -> Ipswich -> Brisbane
# Total Distance: 267 km

# Verification

# Let’s manually calculate the distance for the route OR-Tools provided:
# 	•	Brisbane -> Logan: 27 km
# 	•	Logan -> Gold Coast: 52 km
# 	•	Gold Coast -> Tweed Heads: 28 km
# 	•	Tweed Heads -> Ipswich: 120 km
# 	•	Ipswich -> Brisbane: 40 km

# Total Distance: 27 + 52 + 28 + 120 + 40 = 267 km
# The distance matches the output from the OR-Tools solution.