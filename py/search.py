# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
from util import heappush, heappop
from game import Directions
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
      """
      Returns the start state for the search problem
      """
      util.raiseNotDefined()

    def isGoalState(self, state):
      """
      state: Search state

      Returns True if and only if the state is a valid goal state
      """
      util.raiseNotDefined()

    def getSuccessors(self, state):
      """
      state: Search state

      For a given state, this should return a list of triples,
      (successor, action, stepCost), where 'successor' is a
      successor to the current state, 'action' is the action
      required to get there, and 'stepCost' is the incremental
      cost of expanding to that successor
      """
      util.raiseNotDefined()

    def getCostOfActions(self, actions):
      """
      actions: A list of actions to take

      This method returns the total cost of a particular sequence of actions.  The sequence must
      be composed of legal moves
      """
      util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def constructParentMap(problem):
    queue = [problem.getStartState()]
    key = (problem.getStartState())[0]
    # What you want to construct
    parent_map = dict() # or `{}`

    while len(queue) > 0:
        node = queue[0]
        queue = queue[1:]
        for successor in problem.getSuccessors(node):
            # Where you construct it
            if node[0] not in parent_map:
              parent_map[node[0]] = []
            parent_map[node[0]].append(successor)
    
    parent_map.pop(key)
    return parent_map

def createMap(problem, map):
  start = problem.getStartState()
  

def depthFirstSearch(problem):
    from util import Stack

    closed = []
    fringe = Stack()
    path = []

    #get start node to fringe
    node = problem.getStartState()
    Stack.push(fringe,(node,[]))


    while not fringe.isEmpty():
      #Expand next state in the fringe
      node,path = Stack.pop(fringe)

      #Check if we have found the goal state
      if problem.isGoalState(node): 
        return path

      if node not in closed:
        closed.append(node)

        childNodes = problem.getSuccessors(node)
        for child in childNodes:
          newPath = path + [child[1]]
          Stack.push(fringe,(child[0], newPath))
          
    #If not goal found return empty list
    return []

def breadthFirstSearch(problem):
  from util import Queue

  closed = []
  fringe = Queue()
  path = []

  #get start node to fringe
  node = problem.getStartState()
  Queue.push(fringe,(node,[]))


  while not fringe.isEmpty():
    #Expand next state in the fringe
    node,path = Queue.pop(fringe)

    #Check if we have found the goal state
    if problem.isGoalState(node): 
      return path

    if node not in closed:
      closed.append(node)

      childNodes = problem.getSuccessors(node)
      for child in childNodes:
        newPath = path + [child[1]]
        Queue.push(fringe,(child[0], newPath))
        
  #If not goal found return empty list
  return []

def uniformCostSearch(problem):
  from util import heappush, heappop

  closed = []
  fringe = []
  path = []

  #get start node to fringe
  node = problem.getStartState()
  heappush(fringe,(0,(node,[])))


  while len(fringe) > 0:
    #Expand next state in the fringe
    cost,nodeAndPath = heappop(fringe)
    node = nodeAndPath[0]
    path = nodeAndPath[1]

    #Check if we have found the goal state
    if problem.isGoalState(node): 
      return path

    if node not in closed:
      closed.append(node)

      childNodes = problem.getSuccessors(node)
      for child in childNodes:
        newPath = path + [child[1]] 
        newCost = cost + child[2]
        heappush(fringe,(newCost,(child[0],newPath)))
        
  #If not goal found return empty list
  return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  from util import heappush, heappop

  closed = []
  fringe = []
  path = []

  #get start node to fringe
  node = problem.getStartState()
  cost = heuristic(node,problem) + 0
  ucs = 0
  heappush(fringe,(cost,[node,[],ucs]))


  while len(fringe) > 0:
    #Expand next state in the fringe
    cost,nodeAndPath = heappop(fringe)
    node = nodeAndPath[0]
    path = nodeAndPath[1]
    ucs = nodeAndPath[2]

    #Check if we have found the goal state
    if problem.isGoalState(node):
      return path

    if node not in closed:
      closed.append(node)

      #Get all children
      childNodes = problem.getSuccessors(node)
      for child in childNodes:
        #New path is current path + move to get to child
        newPath = path + [child[1]]
        #New cost = old cost + cost of new node + heuristic
        newUcs = ucs + child[2]
        newCost = newUcs + heuristic(child[0], problem)
        #Add to fringe
        heappush(fringe,(newCost,(child[0],newPath,newUcs)))
        
  #If not goal found return empty list
  return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch