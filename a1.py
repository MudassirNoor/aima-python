#a1
from search import *
import random

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
        for i in range (0, 9):
            eightPuzzleList.append(i)

        random.shuffle(eightPuzzleList)
        eightPuzzleTuple = tuple(eightPuzzleList)
        eightPuzzle = EightPuzzle(eightPuzzleTuple)
        solvable = eightPuzzle.check_solvability(eightPuzzleTuple)

        if solvable:
            return eightPuzzle

        del eightPuzzleTuple, eightPuzzleList, eightPuzzle

def main():
    eightPuzzle = make_rand_8puzzle()
    display(eightPuzzle.initial)

if __name__ == '__main__': main()