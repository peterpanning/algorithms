import heapq

unweighted_graph = {
    'A': [('B', 1), ('C', 1)],
    'B': [('D', 1), ('E', 1)],
    'C': [('F', 1), ('G', 1)],
    'D': [],
    'E': [('D', 1)],
    'F': [],
    'G': []
}  

weighted_graph = {
    'A': [('B', 1), ('C', 2)],
    'B': [('C', 3), ('D', 4)],
    'C': [('D', 2)]
}  

def graphSearch(graph, start, end, fringe):
    """
    Search through the successors of the problem to find a goal. The argument
    fringe should be an empty queue.
    """
    #startState = problem.getStartState()
    fringe.push(Node(start))
    """"
    try:
        startState.__hash__() 
        visited = set()
    except:
        visited = list()
    """
    visited = list()

    while not fringe.isEmpty():
        currentNode = fringe.pop()
        if currentNode.value == end:
            #return [node.action for node in currentNode.nodePath()][1:]
            return currentNode.nodePath()
        """
        try: 
            inVisited = currentNode.state in visited 
        except:
            visited = list(visited)
            inVisited = currentNode.state in visited 
        """
        inVisited = currentNode.value in visited
        if not inVisited:
            #if isinstance(visited, list): 
            visited.append(currentNode.value)
            #else:
            #    visited.add(currentNode.state)
            for node in currentNode.expand(graph):
                fringe.push(node)
    return None

def depthFirstSearch(problem, start, end):
    """Search the deepest nodes in the search tree first."""
    return graphSearch(problem, start, end, Stack())

def breadthFirstSearch(problem, start, end):
    """Search the shallowest nodes in the search tree first."""
    return graphSearch(problem, start, end, Queue())

def uniformCostSearch(problem, start, end):
    """Search the node of least total cost first."""
    return graphSearch(problem, start, end, PriorityQueueWithFunction(lambda x: x.path_cost))

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, start, end, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    return graphSearch(problem, start, end, util.PriorityQueueWithFunction(
        lambda node: node.path_cost + heuristic(
            node.state, problem)))

weighted_graph = {
    'A': [('B', 1), ('C', 2)],
    'B': [('C', 3), ('D', 4)],
    'C': [('D', 2)],
    'D': [('C', 2)]
}    

class Node:
    """ 
    AIMA: A node in a search tree. Contains the value of this node, 
    a pointer to the parent of this node, the path leading to this node, 
    and the total cost of that path. 
    """

    def __init__(self, value, parent=None, path_cost=0):
        # Create a search tree Node, derived from a parent by an action
        self.value = value
        self.parent = parent
        if parent:
            self.path_cost = parent.path_cost + path_cost
            self.depth = parent.depth + 1
        else:
            self.path_cost = path_cost
            self.depth = 0

    def __repr__(self):
        return "<Node %s>" % (self.value,)

    def nodePath(self):
        # Create a list of nodes from the root to this node
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        result.reverse()
        return result

    def expand(self, graph):
        """
        Return a list of nodes reachable from this node
        """

        return [Node(child[0], self, child[1]) for child in graph[self.value]]


class Stack:
    """
    A container with a last-in-first-out (LIFO) queuing policy.
    """
    def __init__(self):
        self.list = []

    def push(self,item):
        "Push 'item' onto the stack"
        self.list.insert(0, item)

    def pop(self):
        "Pop the most recently pushed item from the stack"
        return self.list.pop(0)

    def isEmpty(self):
        "Returns true if the stack is empty"
        return len(self.list) == 0

class Queue:
    "A container with a first-in-first-out (FIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Enqueue the 'item' into the queue"
        self.list.insert(0,item)

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the queue is empty"
        return len(self.list) == 0

class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the user is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        """
        If item already in priority queue with higher priority, update its priority and rebuild the heap.
        If item already in priority queue with equal or lower priority, do nothing.
        If item not in priority queue, do the same thing as self.push.
        """
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

class PriorityQueueWithFunction(PriorityQueue):
    """
    Implements a priority queue with the same push/pop signature of the
    Queue and the Stack classes. This is designed for drop-in replacement for
    those two classes. The caller has to provide a priority function, which
    extracts each item's priority.
    """
    def  __init__(self, priorityFunction):
        "priorityFunction (item) -> priority"
        self.priorityFunction = priorityFunction      # store the priority function
        PriorityQueue.__init__(self)        # super-class initializer

    def push(self, item):
        "Adds an item to the queue with priority from the priority function"
        PriorityQueue.push(self, item, self.priorityFunction(item))
