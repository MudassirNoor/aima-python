#a1
from search import *
import random
import time

def printResult(node, nodesRemoved, elapsedTime):
    print("Final State")
    display(node.state)
    print("Nodes removed: ", nodesRemoved)
    print("Length of solution: ", len(node.solution()))
    print("Time elapsed: ", elapsedTime)

def maximum_heuristic(self, node):
    self : EightPuzzle
    misplacedTileHeuristic = self.h(node)
    manhattanHeuristic = self.manhattan_heuristic(node)

    return max(misplacedTileHeuristic, manhattanHeuristic)

def calculate_manhattanDistance(nodeIndex, goalIndex):
    xInitial = nodeIndex % 3
    yInitial = int(nodeIndex / 3)

    xFinal  = goalIndex % 3
    yFinal = int(goalIndex / 3)

    return abs(xFinal - xInitial) + abs (yFinal - yInitial)

def manhattan_heuristic(self, node):
    self : EightPuzzle
    manhattanDistance = 0

    for i in range(len(self.goal)):
        if (node.state[i] != 0):
            goalPosition = self.goal.index(node.state[i])
            manhattanDistance += calculate_manhattanDistance(i, goalPosition)

    return manhattanDistance

def display(state):
    zerothPosition = state.index(0)
    listRepresentation = list(state)
    listRepresentation[zerothPosition] = '*'
    for i in range(0, 9, 3):
        x, y, z = i, i + 1, i + 2
        print(listRepresentation[x], listRepresentation[y], listRepresentation[z])

def make_rand_8puzzle():
    solvable = False
    while (not solvable):
        eightPuzzleList = []
        for i in range(9):
            eightPuzzleList.append(i)

        random.shuffle(eightPuzzleList)
        eightPuzzleTuple = tuple(eightPuzzleList)
        eightPuzzle = EightPuzzle(eightPuzzleTuple)
        solvable = eightPuzzle.check_solvability(eightPuzzleTuple)

        if solvable:
            return eightPuzzle

        del eightPuzzleTuple, eightPuzzleList, eightPuzzle

def main():
    # Monkey Patching
    # Well aware it is not a good practice, but within the current scope it is fine to use
    EightPuzzle.manhattan_heuristic = manhattan_heuristic
    EightPuzzle.maximum_heuristic = maximum_heuristic

    eightPuzzle = make_rand_8puzzle()
    # eightPuzzle = EightPuzzle((7,2,4,5,0,6,8,3,1), (0,1,2,3,4,5,6,7,8))
    print("Initial State")
    display(eightPuzzle.initial)

    startTime = time.time()
    solved1, nodesRemoved1 = astar_search(eightPuzzle)
    printResult(solved1, nodesRemoved1, time.time() - startTime)

    startTime = time.time()
    solved2, nodesRemoved2 = astar_search(eightPuzzle, eightPuzzle.manhattan_heuristic)
    printResult(solved2, nodesRemoved2, time.time() - startTime)

    startTime = time.time()
    solved3, nodesRemoved3 = astar_search(eightPuzzle, eightPuzzle.maximum_heuristic)
    printResult(solved3, nodesRemoved3, time.time() - startTime)

def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

# Modified the following code to return an additional value
def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    numNodesRemoved = 0
    while frontier:
        node = frontier.pop()
        numNodesRemoved += 1
        if problem.goal_test(node.state):
            if display:
                print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            return node, numNodesRemoved
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
    return None, numNodesRemoved

if __name__ == '__main__': main()