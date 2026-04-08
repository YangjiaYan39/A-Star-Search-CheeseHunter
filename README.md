# Intelligent Path Planning with A* Search (CheeseHunter)

A project focused on improving search efficiency in a multi-goal path planning problem using a custom-designed A* heuristic.

---

## Overview

This project solves a grid-based planning problem where an agent must activate multiple required targets (levers) before reaching the final goal.

Unlike standard shortest-path problems, the agent must visit all required locations under environmental constraints such as obstacles and traps.
This significantly increases the search complexity and makes naive approaches inefficient.

---

## Approach

### Baseline: Uniform Cost Search (UCS)

Uniform Cost Search guarantees optimal solutions but performs poorly due to the large state space.

### Optimized: A* Search

To address this, I implemented A* search with a problem-specific heuristic to guide the search more effectively and reduce unnecessary exploration.

---

## Heuristic Design

The heuristic is designed to estimate the remaining cost of completing all required objectives. It combines:

* Distance from the current position to the nearest unvisited target
* A Minimum Spanning Tree (MST) over all remaining targets
* Distance from the final target to the goal

All distances are computed using obstacle-aware BFS instead of simple Manhattan distance, ensuring the estimate reflects the actual environment.

This results in a tighter and more informative heuristic while maintaining admissibility.

---

## Key Optimizations

* Precomputed BFS distance maps between important points
* Bitmask-based state representation for efficient state tracking
* MST-based lower bound estimation
* Memoization to avoid repeated computations
* Early state pruning to reduce unnecessary expansions

---

## Results

On a sample testcase (`level_1.txt`):

* Path cost: **17.4**
* Nodes expanded: **16**
* Maximum frontier size: **2**

Compared to uninformed search, the heuristic significantly reduces the number of explored states and improves efficiency.

---

## Why This Project Matters

This project demonstrates how domain-specific heuristics can drastically improve search performance.

Instead of relying on simple distance metrics, the solution integrates:

* Local shortest path computation (BFS)
* Global structure estimation (MST)
* Efficient state representation and pruning

These techniques are directly applicable to:

* AI planning systems
* Robotics navigation
* Game AI
* Optimization problems with multiple constraints

---

## Project Structure

* `solution.py` — A* implementation, heuristic design, and optimizations
* `run.py` — entry point for running the solver
* `game_env.py` — environment definition
* `game_state.py` — state representation
* `testcases/` — input levels for testing
* `README.md` — project documentation

---

## How to Run

```bash
git clone https://github.com/YangjiaYan39/A-Star-Search-CheeseHunter
cd A-Star-Search-CheeseHunter
python run.py
```

The default configuration runs:

```python
GameEnv("testcases/level_1.txt")
```

---

## Example Output

```text
Running A* search...
Solution: ['wl', 'sl', 'sl', 'sl', 'c', 'c', 'wr', 'sr', 'sr', 'sr']
Cost: 17.4
Nodes expanded: 16

=== Heuristic Performance ===
Nodes expanded: 16
Frontier max size: 2
```

---

## Technologies

* Python
* A* Search
* Breadth-First Search (BFS)
* Minimum Spanning Tree (MST)
* Priority Queue (heapq)
