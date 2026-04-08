from solution import Solver
from game_env import GameEnv

if __name__ == "__main__":
    env = GameEnv(testcase=1)
    solver = Solver(env)
    print("Running A* search...")
    solution = solver.search_a_star()
    print("Solution:", solution)
    print("Cost:", solver.optimalSolutionCost)
    print("Nodes expanded:", solver.usedNodes)
    print("\n=== Heuristic Performance ===")
    print("Nodes expanded:", solver.usedNodes)
    print("Frontier max size:", solver.maxFrontier)