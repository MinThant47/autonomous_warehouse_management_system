import gurobipy as gp

print("Gurobi version:", gp.gurobi.version())

try:
    m = gp.Model()
    print("Model created successfully.")
except gp.GurobiError as e:
    print(e)