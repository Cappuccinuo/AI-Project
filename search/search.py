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
import game
from util import Stack
from util import Queue
from util import PriorityQueue
from game import Directions

"""
Inspired by the CS188 SP14 Lecture 2 -- Uninformed Search
'Nodes are conceptually paths, but better to represent with 
a state, cost, last action, and reference to the parent node.'
"""

class Node:

    def __init__(self, state, cost = 0, lastAction = []):
        self.state = state
        self.cost = cost
        self.lastAction = lastAction

    def expand(self, problem):
        expandNodes = []
        """
        Function problem.getSuccessors: 
        returns a list of triples (successor, action, stepCost)
        Returns successor states, the actions they require, and a cost of 1.
        """
        for (successorStates, lastAction, cost) in problem.getSuccessors(self.state):
            expandNodes.append(Node(successorStates, cost, lastAction))
        return expandNodes

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



def graphSearch(problem, fringe):

    """
    graphSearch : problem fringe -> solution || failure
    GIVEN: a SearchProblem, and a fringe, put certain data into specific kind of data structure(DFS : Stack(), BFS : Queue())
    RETURNS: a solution, which will be transfered to a list of action, or a failure

    EXAMPLES:
    graphSearch(problem, Stack()) -> dfsNode (dfsNode.state = (1, 1), dfsNode.cost = 1, dfsNode.lastAction = West)

    DESIGN STATEGY: Use the Pseudocode of Graph-Search
    Pseudocode:
    
    function GRAPH-SEARCH(problem, fringe) return a solution, or failure
        closed <- an empty set
        fringe <- INSERT(MAKE-NODE(INITIAL-STATE[problem]), fringe)
        loop do
            if fringe is empty then return failure
            node <- REMOVE-FRONT(fringe)
            if GOAL-TEST(problem, STATE[node]) then return node
            if STATE[node] is not in closed then
                add STATE[node] to closed
                for child-node in EXPAND(STATE[node], problem) do
                    fringe <- INSERT(child-node, fringe)
                end
        end

    """
    closed = set()
    path = []
    
    start = problem.getStartState()
    fringe.push(Node(start, 0, []))

    while not fringe.isEmpty():
        peek = fringe.pop()
        if problem.isGoalState(peek.state):
            return peek.lastAction

        if peek.state not in closed:
            closed.add(peek.state)
            expandNodes = peek.expand(problem)
            for node in expandNodes:
                path = list(peek.lastAction)
                path.append(node.lastAction)
                fringe.push(Node(node.state, node.cost, path))
    return None

def graphSearch_UCS(problem, fringe):

    '''
    If you want the parent map, remember that it is only safe to update the parent
    map when the child is on top of the queue. Only then has the algorithm determined
    the shortest path to the current node
    -- https://stackoverflow.com/questions/43354715/uniform-cost-search-in-python
    '''
    closed = set()
    path = []

    start = problem.getStartState()
    fringe.update(Node(start, 0, []), 0)

    while not fringe.isEmpty():
        peek = fringe.pop()
        if problem.isGoalState(peek.state):
            return peek.lastAction

        if peek.state not in closed:
            closed.add(peek.state)
            expandNodes = peek.expand(problem)
            for node in expandNodes:
                path = list(peek.lastAction)
                path.append(node.lastAction)
                cost = problem.getCostOfActions(path)
                fringe.update(Node(node.state, node.cost, path), cost)
    return None

def graphSearch_aStar(problem, fringe, heuristic):
    closed = set()
    path = []

    start = problem.getStartState()
    pathCost = 0
    heuristicCost = heuristic(start, problem)
    totalCost = pathCost + heuristicCost
    fringe.update(Node(start, 0, []), totalCost)

    while not fringe.isEmpty():
        peek = fringe.pop()
        if problem.isGoalState(peek.state):
            return peek.lastAction

        if peek.state not in closed:
            closed.add(peek.state)
            expandNodes = peek.expand(problem)
            for node in expandNodes:
                path = list(peek.lastAction)
                path.append(node.lastAction)
                pathCost = problem.getCostOfActions(path)
                heuristicCost = heuristic(node.state, problem)
                totalCost = pathCost + heuristicCost
                fringe.update(Node(node.state, node.cost, path), totalCost)
    return None


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    dfsPath = graphSearch(problem, Stack())
    return dfsPath
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    bfsPath = graphSearch(problem, Queue())
    return bfsPath
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    ucsPath = graphSearch_UCS(problem, PriorityQueue())
    return ucsPath
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    aStarPath = graphSearch_aStar(problem, PriorityQueue(), heuristic)
    return aStarPath
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
