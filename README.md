# Intelligent Path Planning with A* Search (CheeseHunter)

A search optimization project for solving a multi-goal grid planning problem using a custom A* heuristic.

Reduced node expansions by over 98% through problem-specific heuristic design.

---

## Overview

This project focuses on solving a grid-based planning problem where an agent must activate all levers before reaching the goal.

Unlike standard shortest-path problems, the agent must visit multiple required locations while navigating obstacles and traps.
This significantly increases the complexity of the search space.

---

## Methods

### Uniform Cost Search (Baseline)

Uniform Cost Search (UCS) guarantees optimal solutions but becomes inefficient as the state space grows.

### A* Search (Optimized)

A* search is implemented with a custom heuristic tailored for this multi-goal setting, significantly reducing unnecessary exploration.

---

## Heuristic Design

The heuristic combines three components:

1. Distance from the current position to the nearest unactivated lever
2. A minimum spanning tree (MST) over all remaining levers
3. Distance from the final lever to the goal

All distances are computed using obstacle-aware BFS rather than Manhattan distance,
ensuring the heuristic reflects actual path constraints.

The heuristic remains admissible while providing a tighter lower bound than simple distance metrics.

---

## Optimization Techniques

* Precomputed BFS distance maps
* Bitmask-based state representation
* MST caching for repeated queries
* Dominance pruning to remove suboptimal states
* Heuristic memoization to avoid recomputation

---

## Results

Performance improvements on large test cases:

* Node expansions reduced from over 4,000,000 to ~67,000
* More than 98% reduction in search space
* Runtime reduced from ~120s to ~4.5s

The improvement becomes more significant as the problem size increases.

---

## Why This Project Matters

This project demonstrates how problem-specific heuristic design can dramatically improve search efficiency.

Instead of relying on generic distance metrics, the solution integrates:

* Obstacle-aware shortest path computation (BFS)
* Global structure estimation (MST)
* State pruning and memoization

These ideas are widely applicable in:

* AI planning systems
* Robotics navigation
* Game AI
* Large-scale optimization problems

---

## Project Structure

* `solution.py` — A* and UCS implementation, heuristic design, pruning, and optimization
* `run.py` — entry point to execute the solver
* `game_env.py` — environment definition required to run the solver
* `game_state.py` — state representation for the environment
* `README.md` — project documentation
* `AStar_Search_Report.pdf` — detailed report

---

## How to Run

Clone the repository and run:

```bash
git clone https://github.com/YangjiaYan39/A-Star-Search-CheeseHunter
cd A-Star-Search-CheeseHunter
python run.py
```

---

## Example Output

```text
Running A* search...
Solution: ['UP', 'RIGHT', 'RIGHT', 'DOWN', ...]
Cost: 40.9
Nodes expanded: 62

=== Heuristic Performance ===
Nodes expanded: 62
Frontier max size: 18
```

---

## Note

This project uses the minimal CheeseHunter environment files required to run the solver:

* `game_env.py`
* `game_state.py`

These files are included only to ensure reproducibility.
The search logic and heuristic design are implemented in `solution.py`.

---

## Technologies Used

* Python
* A* Search
* Breadth-First Search (BFS)
* Minimum Spanning Tree (MST)
* Priority Queue (heapq)
