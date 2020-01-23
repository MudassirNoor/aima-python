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
    eightPuzzleList = [i for i in range(9)]

    while (not solvable):
        random.shuffle(eightPuzzleList)
        eightPuzzle = EightPuzzle(tuple(eightPuzzleList))
        solvable = eightPuzzle.check_solvability(eightPuzzle.initial)

        if solvable:
            return eightPuzzle

"""For the House Puzzle, all indexes in the first row with the exception of the chimney is instantiated with 9"""
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
                print("  ", end = '')
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

    def check_solvability(self, state):
        """Algorithm based on documentation in this website http://kevingong.com/Math/SixteenPuzzle.html"""

        # Checks for the chimney value index as this tile can only move when the blank square is underneath
        zerothPosition = state.index(0)
        if self.goal[1] == 0:
            if zerothPosition == 1:
                if self.goal[5] != state[5]:
                    return False
            else:
                if self.goal[5] != state[1]:
                    return False
        else:
            if zerothPosition == 1:
                if self.goal[1] != state[5]:
                    return False
            else:
                if self.goal[1] != state[1]:
                    return False

        # Zero needs to be within the current
        stateList = list(state)
        if zerothPosition == 1:
            stateList[5] = 0

        inversion = 0
        for i in range(4, len(stateList)):
            for j in range(i + 1, len(stateList)):
                if (stateList[i] > stateList[j]) and stateList[i] != 0 and stateList[j] != 0:
                    inversion += 1

        """From the bottom (ignoring the first row which includes chimney and roof):
            - if blank square in odd row, then the inversions must be even
            - if blank square in even row, then the inversions must be odd"""
        evenInversion = inversion % 2 == 0
        blankInOddRowFromBottom = stateList.index(0) > 7
        if (blankInOddRowFromBottom and evenInversion) or (not blankInOddRowFromBottom and not evenInversion):
            return True
        else:
            return False

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

def print_result(heuristic, lengthOfSolution, nodesRemoved, elapsedTime):
    print(heuristic)
    print("Nodes removed: ", nodesRemoved)
    print("Length of solution: ", lengthOfSolution)
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

"""This helper function is used to generate a random Y puzzle"""
def make_rand_YPuzzle():
    solvable = False
    numbers = [i for i in range(9)]
    ypuzzleList = [9] * 12
    while not solvable:
        random.shuffle(numbers)
        ypuzzleList[1] = numbers[0]
        index = 1
        for i in range(4, 12):
            ypuzzleList[i] = numbers[index]
            index += 1

        ypuzzle = YPuzzle(tuple(ypuzzleList))
        solvable = ypuzzle.check_solvability(ypuzzle.initial)
        if solvable:
            return ypuzzle

def main():
    run_EightPuzzle()
    run_HousePuzzle()

    return 0

def run_HousePuzzle():
    for i in range(12):
        ypuzzle = make_rand_YPuzzle()
        print("Initial State")
        ypuzzle.display(ypuzzle.initial)
        print ("Goal State")
        ypuzzle.display(ypuzzle.goal)

        startTime = time.time()
        solution, misplacedTilesNodes = astar_search(ypuzzle)
        elapsedTime1 = time.time() - startTime
        print_result("Misplaced Tile Heuristic", len(solution.solution()), misplacedTilesNodes, elapsedTime1)

        startTime = time.time()
        solution, manhattanDistanceNodes = astar_search(ypuzzle, ypuzzle.manhattan_heuristic)
        elapsedTime2 = time.time() - startTime
        print_result("Manhattan Distance Heuristic", len(solution.solution()), manhattanDistanceNodes, elapsedTime2)

        startTime = time.time()
        solution, maximumHeuristicNodes = astar_search(ypuzzle, ypuzzle.maximum_heuristic)
        elapsedTime3 = time.time() - startTime
        print_result("Maximum Heuristic", len(solution.solution()), maximumHeuristicNodes, elapsedTime3)

def run_EightPuzzle():
    # Monkey Patching
    EightPuzzle.manhattan_heuristic = manhattan_heuristic
    EightPuzzle.maximum_heuristic = maximum_heuristic

    for i in range(12):
        eightPuzzle = make_rand_8puzzle()
        print("Initial State")
        display(eightPuzzle.initial)
        print ("Goal State")
        display(eightPuzzle.goal)

        startTime = time.time()
        solution, misplacedTilesNodes = astar_search(eightPuzzle)
        elapsedTime1 = time.time() - startTime
        print_result("Misplaced Tile Heuristic", len(solution.solution()), misplacedTilesNodes, elapsedTime1)

        startTime = time.time()
        solution, manhattanDistanceNodes = astar_search(eightPuzzle, eightPuzzle.manhattan_heuristic)
        elapsedTime2 = time.time() - startTime
        print_result("Manhattan Distance Heuristic", len(solution.solution()), manhattanDistanceNodes, elapsedTime2)

        startTime = time.time()
        solution, maximumHeuristicNodes = astar_search(eightPuzzle, eightPuzzle.maximum_heuristic)
        elapsedTime3 = time.time() - startTime
        print_result("Maximum Heuristic", len(solution.solution()), maximumHeuristicNodes, elapsedTime3)

def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

# Modified the following code to return an additional value:- number of nodes removed from frontier
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