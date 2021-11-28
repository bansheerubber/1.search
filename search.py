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

import util, heapq, time
from game import Directions

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
    fringe.push((start, None, None), heuristic(start, problem, None))

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
                        g_cost + heuristic(successor, problem, None) # only difference between A* and UCS
                    )

def pr(f_score, g_score):
    return max(f_score, 2 * g_score)

class CostMap:
    def __init__(self):
        self.heap = []
        self.map = {}
        self.seen_map = {}
        self.count = 0
    
    def __get_entry(self, node, g_cost):
        return (g_cost, self.seen_map[node], node)

    def update(self, node, g_cost):
        if node not in self.map:
            self.seen_map[node] = self.count
            self.count += 1

            heapq.heappush(self.heap, self.__get_entry(node, g_cost))
            self.map[node] = (self.heap.index(self.__get_entry(node, g_cost)), g_cost)
        else:
            self.heap[self.map[node][0]] = self.__get_entry(node, g_cost)
            heapq.heapify(self.heap)

    def has(self, node):
        return node in self.map

    def get(self, node):
        return self.map[node][1]
    
    def get_min(self):
        return (self.heap[0][0], self.heap[0][2])

def biDirectionalRound(problem, heuristic, goal, a_open, a_closed, b_open, b_closed, a_g, b_g, a_f, a_came_from, last_node, cheapest_solution, reverse):
    n = a_open.pop()
    a_closed.append(n)

    successors = problem.getSuccessors(n, reverse)
    for c, successor_direction, successor_cost in successors:
        if (a_open.has(c) or c in a_closed) and a_g.has(c) and a_g.get(c) <= a_g.get(n) + successor_cost:
            continue
    
        if c in a_closed: # dont bother removing from open, since we're going to update its value
            a_closed.remove(c)

        a_came_from[c] = (n, successor_direction, successor_cost)

        a_g.update(c, a_g.get(n) + successor_cost)
        a_f.update(c, a_g.get(c) + heuristic(c, problem, goal, reverse))
        a_open.update(
            c,
            pr(a_f.get(c), a_g.get(c)),
            a_g.get(c)
        )

        if b_open.has(c) and a_g.has(c) and b_g.has(c):
            cheapest_solution = min(cheapest_solution, a_g.get(c) + b_g.get(c))
            last_node = c
    return (last_node, cheapest_solution)

def biDirectionalSearch(problem, heuristic=nullHeuristic):
    closed_forwards = []
    open_forwards = util.PriorityQueue() # priority determined by pr()

    closed_backwards = []
    open_backwards = util.PriorityQueue() # priority determined by pr()

    g_cost_forwards = CostMap()
    g_cost_backwards = CostMap()

    f_cost_forwards = CostMap()
    f_cost_backwards = CostMap()

    start = problem.getStartState()
    goal = problem.goal
    
    # create initial state
    g_cost_forwards.update(start, 0)
    g_cost_backwards.update(goal, 0)

    f_cost_forwards.update(start, heuristic(start, problem, goal))
    f_cost_backwards.update(goal, heuristic(goal, problem, start, True))

    open_forwards.push(start, 0, g_cost_forwards.get(start))
    open_backwards.push(goal, 0, g_cost_backwards.get(goal))

    cheapest_solution = 1000000000

    came_from_forwards = {}
    came_from_backwards = {}

    last_node = None

    while not open_forwards.isEmpty() and not open_backwards.isEmpty():
        min_forwards = open_forwards.heap[0]
        min_backwards = open_backwards.heap[0]
        
        chosen_round = min(min_forwards[0], min_backwards[0])

        if cheapest_solution <= max(chosen_round, f_cost_forwards.get_min()[0], f_cost_backwards.get_min()[0], g_cost_forwards.get_min()[0] + g_cost_backwards.get_min()[0] + 0.00001):
            directions = []

            problem.getSuccessors(last_node) # force the bridge to draw as expanded nodes
            problem.isGoalState(goal) # force the expanded nodes to draw

            current = last_node
            while current in came_from_backwards: # construct backwards list
                current, direction, _ = came_from_backwards[current]
                if direction:
                    directions.insert(0, Directions.REVERSE[direction])
            directions.reverse()

            current = last_node
            while current in came_from_forwards: # construct forwards list
                current, direction, _ = came_from_forwards[current]
                if direction:
                    directions.insert(0, direction)

            return directions

        if chosen_round == min_forwards[0]:
            last_node, cheapest_solution = biDirectionalRound(
                problem,
                heuristic,
                goal,
                open_forwards,
                closed_forwards,
                open_backwards,
                closed_backwards,
                g_cost_forwards,
                g_cost_backwards,
                f_cost_forwards,
                came_from_forwards,
                last_node,
                cheapest_solution,
                False
            )
        elif chosen_round == min_backwards[0]:
            last_node, cheapest_solution = biDirectionalRound(
                problem,
                heuristic,
                start,
                open_backwards,
                closed_backwards,
                open_forwards,
                closed_forwards,
                g_cost_backwards,
                g_cost_forwards,
                f_cost_backwards,
                came_from_backwards,
                last_node,
                cheapest_solution,
                True
            )

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
bi = biDirectionalSearch
