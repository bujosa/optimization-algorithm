from ortools.init.python import init
from ortools.linear_solver import pywraplp
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Create the linear solver with the GLOP backend.
solver = pywraplp.Solver.CreateSolver("GLOP")
if not solver:
    print("Could not create solver GLOP")
    exit(1)

# Create the variables x and y.
x_var = solver.NumVar(0, 1, "x")
y_var = solver.NumVar(0, 2, "y")

print("Number of variables =", solver.NumVariables())

infinity = solver.infinity()
# Create a linear constraint, x + y <= 2.
constraint = solver.Constraint(-infinity, 2, "ct")
constraint.SetCoefficient(x_var, 1)
constraint.SetCoefficient(y_var, 1)

print("Number of constraintss =", solver.NumConstraints())

# Create the objective function, 3 * x + y.
objective = solver.Objective()
objective.SetCoefficient(x_var, 3)
objective.SetCoefficient(y_var, 1)
objective.SetMaximization()

print(f"Solving with {solver.SolverVersion()}")
result_status = solver.Solve()
print(f"Status: {result_status}")
if result_status != pywraplp.Solver.OPTIMAL:
    print("The problem does not have an optimal solution!")
    if result_status == pywraplp.Solver.FEASIBLE:
        print("A potentially suboptimal solution was found")
    else:
        print("The solver could not solve the problem.")
        exit(1)

print("Solution:")
print("Objective value =", objective.Value())
print("x =", x_var.solution_value())
print("y =", y_var.solution_value())