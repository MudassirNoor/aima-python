#a1
from search import *
import random
import time

def display(state):
    zerothPosition = state.index(0)
    listRepresentation = list(state)
    listRepresentation[zerothPosition] = '*'
    for i in range(9):
        if i != 0 and i % 3 == 0:
            print()
        print(str(listRepresentation[i]) + " " , end = '')

    print()

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

class YPuzzle(Problem):
    def __init__(self, initial, goal =(9,1,9,9,2,3,4,5,6,7,8,0)):
        """For initializing this puzzle, ensure that indexes 0, 2 and 3 are instantiated with 9"""

        super().__init__(initial, goal)

    def display(self, state):
        zerothPosition = state.index(0)
        listRepresentation = list(state)
        listRepresentation[zerothPosition] = '*'
        for i in range(12):
            if i != 0 and i % 4 == 0:
                print()
            if listRepresentation[i] == 9:
                print(" ", end = '')
            else:
                print(str(listRepresentation[i]) + ' ', end = '')

        print()

    def actions(self, state):
        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']

        index_blank_square = self.find_blank_square(state)
        row = int(index_blank_square / 4)
        column = index_blank_square % 4

        atChimney = index_blank_square == 1
        if atChimney:
            possible_actions = ['DOWN']
            return possible_actions

        if column == 0:
            possible_actions.remove('LEFT')
        if row == 0 or (row == 1 and column != 1):
            possible_actions.remove('UP')
        if column == 3:
            possible_actions.remove('RIGHT')
        if row == 2:
            possible_actions.remove('DOWN')

        return possible_actions

    # Slight modification made to the code in EightPuzzle class
    def result(self, state, action):
        blank = self.find_blank_square(state)
        new_state = list(state)

        delta = {'UP': -4, 'DOWN': 4, 'LEFT': -1, 'RIGHT': 1}
        neighbor = blank + delta[action]
        new_state[blank], new_state[neighbor] = new_state[neighbor], new_state[blank]

        return tuple(new_state)

    def find_blank_square(self, state):
        return state.index(0)

    #TODO: def check_solvability(self, state):

    def h(self, node):
        """Misplaced tile heuristic"""
        misplacedTiles = 0
        for i in range(len(node.state)):
            if (node.state[i] != 9 and node.state[i] != 0):
                if (node.state[i] != self.goal[i]):
                    misplacedTiles += 1

        return misplacedTiles

    def manhattan_heuristic(self, node):
        manhattanDistance = 0
        for i in range(len(self.goal)):
            if (node.state[i] != 0 and node.state[i] != 9):
                goalPosition = self.goal.index(node.state[i])
                manhattanDistance += self.calculate_manhattanDistance(i, goalPosition)

        return manhattanDistance

    def maximum_heuristic(self, node):
        misplacedTileHeuristic = self.h(node)
        manhattanHeuristic = self.manhattan_heuristic(node)

        return max(misplacedTileHeuristic, manhattanHeuristic)

    def calculate_manhattanDistance(self, nodeIndex, goalIndex):
        xInitial = nodeIndex % 4
        yInitial = int(nodeIndex / 4)

        xFinal = goalIndex % 4
        yFinal = int(goalIndex / 4)

        return abs(xFinal - xInitial) + abs(yFinal - yInitial)

"""These helper functions are specific to the Eight Puzzle problems
    - print_result
    - maximum_heuristic
    - manhattan_heuristic
    - calculate_manhattanDistance"""

def print_result(node, nodesRemoved, elapsedTime):
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

def manhattan_heuristic(self, node):
    self : EightPuzzle
    manhattanDistance = 0

    for i in range(len(self.goal)):
        if node.state[i] != 0:
            goalPosition = self.goal.index(node.state[i])
            manhattanDistance += calculate_manhattanDistance(i, goalPosition)

    return manhattanDistance

def calculate_manhattanDistance(nodeIndex, goalIndex):
    xInitial = nodeIndex % 3
    yInitial = int(nodeIndex / 3)

    xFinal  = goalIndex % 3
    yFinal = int(goalIndex / 3)

    return abs(xFinal - xInitial) + abs (yFinal - yInitial)

def main():
    run_EightPuzzle()
    # run_HousePuzzle()


def run_HousePuzzle():
    # ypuzzle = YPuzzle((9,1,9,9,2,0,4,5,6,3,7,8))
    # ypuzzle = YPuzzle((9,0,9,9,2,1,8,4,6,3,7,5))
    ypuzzle = YPuzzle(())
    print("Initial State")
    ypuzzle.display(ypuzzle.initial)

    startTime = time.time()
    solved1, nodesRemoved1 = astar_search(ypuzzle)
    elapsedTime = time.time() - startTime
    ypuzzle.display(solved1.state)
    print("Nodes removed: ", nodesRemoved1)
    print("Length of solution: ", len(solved1.solution()))
    print("Time elapsed: ", elapsedTime)

    # startTime = time.time()
    # solved2, nodesRemoved2 = astar_search(ypuzzle, ypuzzle.manhattan_heuristic)
    # elapsedTime = time.time() - startTime
    # ypuzzle.display(solved2.state)
    # print("Nodes removed: ", nodesRemoved2)
    # print("Length of solution: ", len(solved2.solution()))
    # print("Time elapsed: ", elapsedTime)
    #
    # startTime = time.time()
    # solved3, nodesRemoved3 = astar_search(ypuzzle, ypuzzle.maximum_heuristic)
    # elapsedTime = time.time() - startTime
    # ypuzzle.display(solved3.state)
    # print("Nodes removed: ", nodesRemoved3)
    # print("Length of solution: ", len(solved3.solution()))
    # print("Time elapsed: ", elapsedTime)

    return 0

def run_EightPuzzle():
    # Monkey Patching
    # Well aware it is not a good practice, but within the current scope it is fine to use
    EightPuzzle.manhattan_heuristic = manhattan_heuristic
    EightPuzzle.maximum_heuristic = maximum_heuristic

    eightPuzzle = EightPuzzle((7,2,4,5,0,6,8,3,1), (0,1,2,3,4,5,6,7,8))
    print("Initial State")
    display(eightPuzzle.initial)

    startTime = time.time()
    solved1, nodesRemoved1 = astar_search(eightPuzzle)
    print_result(solved1, nodesRemoved1, time.time() - startTime)

    startTime = time.time()
    solved2, nodesRemoved2 = astar_search(eightPuzzle, eightPuzzle.manhattan_heuristic)
    print_result(solved2, nodesRemoved2, time.time() - startTime)

    startTime = time.time()
    solved3, nodesRemoved3 = astar_search(eightPuzzle, eightPuzzle.maximum_heuristic)
    print_result(solved3, nodesRemoved3, time.time() - startTime)

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