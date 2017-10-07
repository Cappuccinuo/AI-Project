# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)

        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        foodScore = 10.0

        newGhostPositions = successorGameState.getGhostPositions()
        minDistanceToGhost = float('inf')

        for ghostPos in newGhostPositions:
            distanceToGhost = manhattanDistance(newPos, ghostPos)
            if distanceToGhost > 0:
                if distanceToGhost < minDistanceToGhost:
                    minDistanceToGhost = distanceToGhost

        
        minDistanceToFood = float('inf')
        newFoodPositions =  newFood.asList()
        for newFoodPosition in newFoodPositions:
            distanceToFood = manhattanDistance(newPos, newFoodPosition)
            if distanceToFood < minDistanceToFood:
              minDistanceToFood = distanceToFood

        heuristic = successorGameState.getScore() + foodScore / minDistanceToFood - foodScore / minDistanceToGhost
        return heuristic

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

    def isTerminal(self, state, depth):
        return self.depth == depth or state.isWin() or state.isLose()

    def isPacman(self, agentIndex):
        return agentIndex == 0

    def isNextDepth(self, state, agentIndex):
        return agentIndex == state.getNumAgents()


class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game

          gameState.isWin():
            Returns whether or not the game state is a winning state

          gameState.isLose():
            Returns whether or not the game state is a losing state
        """

        
        def minimax(state, depth, agentIndex):
            if self.isTerminal(state, depth):
                return self.evaluationFunction(state)

            if self.isNextDepth(state, agentIndex):  
                return minimax(state, depth + 1, 0)  

            actions = state.getLegalActions(agentIndex)
            results = [
                minimax(state.generateSuccessor(agentIndex, action), depth, agentIndex + 1)
                for action in actions]
            
            if self.isPacman(agentIndex):
                return max(results)
            else:
                return min(results)

        pacmanActions = gameState.getLegalActions(0)
        best_score = float('-inf')
        best_move = Directions.STOP
        for action in pacmanActions:
            successor = gameState.generateSuccessor(0, action)
            score = minimax(successor, 0, 1)
            if score > best_score:
                best_score = score
                best_move = action        
        return best_move


class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        
        def prune(state, depth, agentIndex, A=float("-inf"), B=float("inf")):
            if self.isTerminal(state, depth):
                return self.evaluationFunction(state), None

            if self.isNextDepth(state, agentIndex):
                return prune(state, depth + 1, 0, A, B)

            if self.isPacman(agentIndex):
                best_score = float("-inf")
            else:
                best_score = float("inf")
            best_move = Directions.STOP

            actions = state.getLegalActions(agentIndex)
            for action in actions:
                successor = state.generateSuccessor(agentIndex, action)
                score, move = prune(successor, depth, agentIndex + 1, A, B)
                best_score, best_move = (max if agentIndex == 0 else min)((best_score, best_move), (score, action))

                if (self.isPacman(agentIndex)):
                    if best_score > B:
                        return best_score, best_move
                    A = max(A, best_score)
                else:
                    if best_score < A:
                        return best_score, best_move
                    B = min(B, best_score)
            return best_score, best_move

        score, move = prune(gameState, 0, 0)
        return move
        


class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        def expectimax(state, depth, agentIndex):
            if self.isTerminal(state, depth):
                return self.evaluationFunction(state)

            if self.isNextDepth(state, agentIndex):
                return expectimax(state, depth + 1, 0)

            actions = state.getLegalActions(agentIndex)
            results = [
                expectimax(state.generateSuccessor(agentIndex, action), depth, agentIndex + 1)
                for action in actions
            ]
            
            if (self.isPacman(agentIndex)):
                return max(results)
            else:
                return sum(results) * 1.0 / len(results)

        pacmanActions = gameState.getLegalActions(0)
        best_score = float("-inf")
        best_move = Directions.STOP
        for action in pacmanActions:
            successor = gameState.generateSuccessor(0, action)
            score = expectimax(successor, 0, 1)
            if score > best_score:
                best_score = score
                best_move = action
        return best_move


def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: <write something here so we know what you did>

      distance to the closest active ghost
      distance to the closest scared ghost
      distance to the closest food
      number of foods left
      number of capsules left
    """

    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood()
    newGhostStates = currentGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
    currentScore = currentGameState.getScore()

    if currentGameState.isLose():
        return float("-inf")
    if currentGameState.isWin():
        return float("inf")

    minDisToFood = float("inf")
    newFoodPositions = newFood.asList()
    for foodPos in newFoodPositions:
        distanceToFood = manhattanDistance(newPos, foodPos)
        if distanceToFood < minDisToFood:
            minDisToFood = distanceToFood

    numOfFoodLeft = len(newFoodPositions)

    numOfCapsulesLeft = len(currentGameState.getCapsules())

    scaredGhosts, terrorGhosts = [], []
    for ghostState in newGhostStates:
        if ghostState.scaredTimer:
            scaredGhosts.append(ghostState)
        else:
            terrorGhosts.append(ghostState)

    minDisToScaredGhost = 0
    minDisToTerrorGhost = float("inf")

    if scaredGhosts:
        for scaredGhost in scaredGhosts:
            distanceToScaredGhost = manhattanDistance(newPos, scaredGhost.getPosition())
            if distanceToScaredGhost < minDisToScaredGhost:
                minDisToScaredGhost = distanceToScaredGhost

    if terrorGhosts:
        for terrorGhost in terrorGhosts:
            distanceToTerroGhost = manhattanDistance(newPos, terrorGhost.getPosition())
            if distanceToTerroGhost < minDisToTerrorGhost:
                minDisToTerrorGhost = distanceToTerroGhost


    score = currentScore + 10.0 / minDisToFood - 20.0 / minDisToTerrorGhost - 200 * minDisToScaredGhost - 100 * numOfCapsulesLeft - 25 * numOfFoodLeft
    return score


# Abbreviation
better = betterEvaluationFunction

