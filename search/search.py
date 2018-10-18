class Node:
    """ 
    AIMA: A node in a search tree. Contains a pointer to the parent of this node, 
    the state of this node, the action that got us to this node, the path leading to
    this node, and the total cost of that path. 
    """

    def __init__(self, state, parent=None, action=None, path_cost=0):
        # Create a search tree Node, derived from a parent by an action
        self.state = state
        self.parent = parent
        self.action = action 
        if parent:
            self.path_cost = parent.path_cost + path_cost
            self.depth = parent.depth + 1
        else:
            self.path_cost = path_cost
            self.depth = 0

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def nodePath(self):
        # Create a list of nodes from the root to this node
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        result.reverse()
        return result

    def expand(self, problem):
        """
        Return a list of nodes reachable from this node
        """
        return [Node(next, self, act, cost)
                for (next, act, cost) in problem.getSuccessors(self.state)]

def graphSearch(problem, fringe):
    """
    Search through the successors of the problem to find a goal. The argument
    fringe should be an empty queue.
    """
    startState = problem.getStartState()
    fringe.push(Node(startState))
    try:
        startState.__hash__() 
        visited = set()
    except:
        visited = list()

    while not fringe.isEmpty():
        currentNode = fringe.pop()
        if problem.isGoalState(currentNode.state):
                return [node.action for node in currentNode.nodePath()][1:]
        try: 
            inVisited = currentNode.state in visited 
        except:
            visited = list(visited)
            inVisited = currentNode.state in visited 
        if not inVisited:
            if isinstance(visited, list): 
                visited.append(currentNode.state)
            else:
                visited.add(currentNode.state)
            for node in currentNode.expand(problem):
                fringe.push(node)
    return None

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    """
    return graphSearch(problem, util.Stack())

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    return graphSearch(problem, util.Queue())

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    return graphSearch(problem, util.PriorityQueueWithFunction(lambda x: x.path_cost))

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    return graphSearch(problem, util.PriorityQueueWithFunction(
        lambda node: node.path_cost + heuristic(
            node.state, problem)))