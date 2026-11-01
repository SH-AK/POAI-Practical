"""A* search: find the shortest path through a weighted graph.

Run with: python a_star.py
Use non-negative edge costs and heuristic estimates that never overestimate
the remaining cost to the goal. The heuristic at the goal must be zero.
"""

import heapq
from math import inf


def a_star(graph, heuristic, start, goal):
    """Return (shortest_path, total_cost), or ([], inf) if no path exists.

    graph maps each node to a dictionary of neighbors and edge costs.
    heuristic maps nodes to estimated costs to the goal; missing values use 0.
    """
    if any(cost < 0 for neighbors in graph.values() for cost in neighbors.values()):
        raise ValueError("A* requires non-negative edge costs.")

    # f(n) = g(n) + h(n)
    # g(n): actual cost from the start; h(n): estimated cost to the goal.
    frontier = [(heuristic.get(start, 0), 0, start)]
    best_cost = {start: 0}
    came_from = {}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)

        # Ignore an older queue entry if a cheaper route has been found.
        if current_cost != best_cost[current]:
            continue

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, current_cost

        for neighbor, edge_cost in graph.get(current, {}).items():
            new_cost = current_cost + edge_cost
            if new_cost < best_cost.get(neighbor, inf):
                best_cost[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic.get(neighbor, 0)
                heapq.heappush(frontier, (priority, new_cost, neighbor))

    return [], inf


def main():
    graph = {
        "A": {"B": 1, "C": 4},
        "B": {"D": 2, "E": 3},
        "C": {"F": 3},
        "D": {"F": 4},
        "E": {"F": 2},
        "F": {},
    }

    # Estimated remaining costs to the goal F.
    heuristic = {"A": 5, "B": 4, "C": 3, "D": 4, "E": 2, "F": 0}
    start, goal = "A", "F"

    path, total_cost = a_star(graph, heuristic, start, goal)
    if path:
        print(f"Shortest path from {start} to {goal}: {' -> '.join(path)}")
        print(f"Total cost: {total_cost}")
    else:
        print(f"No path found from {start} to {goal}.")


if __name__ == "__main__":
    main()
