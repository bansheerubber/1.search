# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    closed = []
    came_from = {}
    fringe = util.Stack()
    fringe.push((problem.getStartState(), None, None))

    while True:
        if fringe.isEmpty():
            print("*** DFS failure")
            return []

        node, direction, _ = fringe.pop()

        if problem.isGoalState(node):
            directions = []
            current = node
            while current in came_from:
                current, direction, _ = came_from[current]
                if direction:
                    directions.insert(0, direction)
            return directions

        closed.append(node)
        
        successors = problem.getSuccessors(node)
        for successor, successor_direction, __ in successors:
            if successor not in closed:
                came_from[successor] = (node, successor_direction, _)
                fringe.push((successor, successor_direction, __))

def breadthFirstSearch(problem):
    came_from = {}
    fringe = util.Queue()
    start = (problem.getStartState(), None, None)
    fringe.push(start)
    frontier = [start]
    closed = set()

    while True:
        if fringe.isEmpty():
            print("*** BFS failure")
            return []

        state, direction, _ = fringe.pop()

        if problem.isGoalState(state):
            directions = []
            current = state
            while current in came_from:
                current, direction, _ = came_from[current]
                if direction:
                    directions.insert(0, direction)
            return directions

        closed.add(state)

        successors = problem.getSuccessors(state)
        for successor, successor_direction, __ in successors:
            if successor not in closed:
                came_from[successor] = (state, successor_direction, _)
                fringe.push((successor, successor_direction, __))
                closed.add(successor)

def uniformCostSearch(problem):
    closed = []
    came_from = {}
    fringe = util.PriorityQueue()
    start = problem.getStartState()
    fringe.push((start, None, None), 0)

    remembered_cost = {}
    remembered_cost[start] = 0

    while True:
        if fringe.isEmpty():
            print("*** A* failure")
            return []

        node, direction, cost = fringe.pop()

        if problem.isGoalState(node):
            directions = []
            current = node
            while current in came_from:
                current, direction, _ = came_from[current]
                if direction:
                    directions.insert(0, direction)
            return directions

        closed.append(node)
        
        successors = problem.getSuccessors(node)
        for successor, successor_direction, successor_cost in successors:
            g_cost = remembered_cost[node] + successor_cost
            if successor not in remembered_cost or g_cost < remembered_cost[successor]:
                came_from[successor] = (node, successor_direction, cost)
                remembered_cost[successor] = g_cost
                if successor not in closed:
                    fringe.update(
                        (successor, successor_direction, successor_cost),
                        g_cost
                    )

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    closed = []
    came_from = {}
    fringe = util.PriorityQueue()
    start = problem.getStartState()
    fringe.push((start, None, None), heuristic(start, problem))

    remembered_cost = {}
    remembered_cost[start] = 0

    while True:
        if fringe.isEmpty():
            print("*** A* failure")
            return []

        node, direction, _ = fringe.pop()

        if problem.isGoalState(node):
            directions = []
            current = node
            while current in came_from:
                current, direction, _ = came_from[current]
                if direction:
                    directions.insert(0, direction)
            return directions

        closed.append(node)
        
        successors = problem.getSuccessors(node)
        for successor, successor_direction, successor_cost in successors:
            g_cost = remembered_cost[node] + successor_cost
            if successor not in remembered_cost or g_cost < remembered_cost[successor]:
                came_from[successor] = (node, successor_direction, _)
                remembered_cost[successor] = g_cost
                if successor not in closed:
                    fringe.update(
                        (successor, successor_direction, successor_cost),
                        g_cost + heuristic(successor, problem) # only difference between A* and UCS
                    )


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
