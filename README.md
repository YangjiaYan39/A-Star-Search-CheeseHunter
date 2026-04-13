# Intelligent Path Planning with A* Search (CheeseHunter)

Focuses on heuristic design and performance optimization in large-scale search problems.
Reduced search space by over 98% using a custom A* heuristic combining BFS and MST.

Designed for multi-goal path planning under constraints with significant performance improvements over UCS.

---

## Overview

This project solves a grid-based planning problem where an agent must activate multiple required targets (levers) before reaching the final goal.

Unlike standard shortest-path problems, the agent must visit all required locations under environmental constraints such as obstacles and traps.
This significantly increases the search complexity and makes naive approaches inefficient.

---

## Key Contributions

- Designed a problem-specific A* heuristic combining BFS distance and MST estimation
- Reduced node expansions from 4.2M to 67K on large maps (~98% reduction)
- Implemented bitmask-based state encoding for efficient search
- Applied dominance pruning and memoization to reduce redundant exploration
- Precomputed shortest paths using BFS for fast heuristic evaluation
- Designed the system with scalable state representation and efficient search strategies

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

## Performance

| Algorithm | Nodes Expanded | Runtime |
|----------|--------------|--------|
| UCS      | 4,200,000+   | ~120s  |
| A*       | ~67,000      | ~4.5s  |

The optimized A* significantly reduces both runtime and memory usage compared to UCS.
The results demonstrate the impact of heuristic quality on both time and space complexity.

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

This project highlights the importance of combining algorithm design with practical system optimization.
---

## Project Structure

* `solution.py` — A* implementation, heuristic design, and optimizations
* `run.py` — entry point for running the solver
* `game_env.py` — environment definition
* `game_state.py` — state representation
* `testcases/` — input levels for testing
* `README.md` — project documentation

---

## Acknowledgement

This project is based on the COMP3702 Artificial Intelligence assignment.

The environment and support code (e.g., `game_env.py`, `game_state.py`, `testcases`) were provided by the course.

My contribution focuses on implementing and optimizing the search algorithm in `solution.py`, including heuristic design and performance improvements.

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
