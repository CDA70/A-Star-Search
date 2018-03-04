import heapq
import helpers
import math

# https://docs.python.org/3.4/library/asyncio-queue.html#asyncio.PriorityQueue
# ref: Python cookbook: Recipes for mastering Python 3
class PriorityQueue:
    
    def __init__(self):
        self.items = []
        
    def empty(self):
        return len(self.items) == 0
    
    def add_node(self, item, priority):
        heapq.heappush(self.items, (priority, item))
        
    def pop_node(self):
        return heapq.heappop(self.items)[1]
    
""" 
   method name: getNodes(M, node)
   Description: get all neighbours of node
      param 1: M    = graph
      param 2: node = the node where you need the neighbours from
      RETURN: list, with all neighbours, which are basically nodes
"""
def getNodes(M, node):
    n = M.roads[node]
    return n

""" 
   method name: h_between2Points(M, node, goal)
   Description: Get the heuristic value between two nodes
      param 1: M    = graph
      param 2: node = the start node for the cost
      param 3: goal = the end node for the cost
      RETURN: a float, the heuristic value between 2 nodes
"""
def h_Between2Points(M, node, goal):
    G = M._graph
    x0, y0 = G.node[node]['pos']
    x1, y1 = G.node[goal]['pos']
    return math.sqrt( (x0 - x1)**2 + (y0 - y1)**2 )

""" 
   method name: reconstruct_path(came_from, current)
   Description: it reconstructs the path starting from the goal
      param 1: came_from = a dictionary the contain parent and child nodes
      param 2: current = In the this case the current is the actual goal node
      RETURN: a list with the shortest path
"""
def reconstruct_path(came_from, current):
    shortest_path = [current]
    while current in came_from:
        current = came_from[current]
        shortest_path.append(current)
    shortest_path.reverse()
    return shortest_path


""" 
   method name: shortest_path(M, start, goal)
   Description: it reconstructs the path starting from the goal
      param 1: M     = graph
      param 2: start = the node where you want to start from
      param 3: goal  = the goal or the node you like to go to
      RETURN: none
"""
def shortest_path(M, start, goal):
    # a regular queue sortes by FIFO or LIFO, a priority queue retrieves elements by priority
    # add_node will add a node with priority to the frontier
    # pop_node will return the node with the lowest value and pops it off the list
    
    # 1. initialize frontier
    frontier = PriorityQueue()
    
    # 2. add the start node to the frontier
    frontier.add_node(start, 0)
    
    # 3. initailise a dictionarie came_from and cost_so_far
    came_from = {}
    cost_so_far = {}
    cost_so_far[start] = 0 
    
    # keep going, as long as there a values in the frontier
    while not frontier.empty():
        # the current node is the node the frontier with the lowest value 
        current = frontier.pop_node()
        
        # reconstruct the path as soon as we reach the goal
        if current == goal:
            return reconstruct_path(came_from, current)
        
        # the function getNodes will get all neighbours
        for next in getNodes(M,current):
            # The new cost = g_cost and is simply the current cost so far + the cost between the current node and next node
            new_cost = cost_so_far[current] + h_Between2Points(M,current,next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + h_Between2Points(M,next,goal)
                # assign a priority value to the priority queue
                frontier.add_node(next, priority)
                came_from[next] = current
                
    return None


            

    
    
