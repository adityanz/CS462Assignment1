### graph.py
### an implementation of a graph using an adjacency list.
### Aditya Dixit

## helper class representing a node in a graph. For the moment, nodes
## only have names. Later, we will add state variables.
import queue
from collections import deque

class Node() :
    def __init__(self, n):
        self.name = n

    def __str__(self):
        return str(self.name)

    def __hash__(self):
        return hash(self.name)

### an edge is a link between two nodes. Right now, the only other
### information an edge carries is the weight of the link. Later we
### will add other annotations.

class Edge() :
    def __init__(self, src,dest, weight) :
        self.src = src
        self.dest = dest
        self.weight = weight

    def __str__(self):
        return str(self.src) + str(self.dest) + str(self.weight)

### The graph class itself.
### The nodeTable is a dictionary that maps names to Node objects.
### (this keeps us from having to repeatedly search edgeMap.keys())

### The edgeMap is a dictionary that maps nodes to lists of Edges emanating from that node.

class Graph() :

    def __init__(self):
        self.nodeTable = {}
        self.edgeMap = {}

    ### implements the 'in' keyword. Returns true if the node is in the graph.
    def __contains__(self, item):
        return item in self.nodeTable

    def __str__(self):
        return str(self.edgeMap) + str(self.nodeTable)

    def getNode(self, src):
        return self.nodeTable[src]

    def addNode(self, src):
        if src not in self.nodeTable :
            self.nodeTable[src] = Node(src)

    def addEdge(self, src, dest, weight):
        e = Edge(src,dest,weight)
        self.addNode(src)
        self.addNode(dest)
        if src in self.edgeMap :
            self.edgeMap[src].append(e)
        else :
            self.edgeMap[src] = [e]

    ## Assume file is in the mtx format: % is a comment
    ## Otherwise it's source destination weight
    ### The file in the github repo will work as a sample for you.
    ### It's in the format: source, vertex, weight. You should assume that the graph is symmetric -
    ### if there's an edge from a to b, there's an edge from b to a.
    ### You can find lots of others here: http://networkrepository.com/index.php
    def readFromFile(self, fname):
        print("Reading from file")
        with open(fname) as f :
            for l in f.readlines() :
                if not l.startswith("%") :
                    (s,d,w) = l.split()
                    #changed from the char to int
                    s = int(s)
                    d = int(d)
                    w = int(w)
                    self.addEdge(s,d,w)

    ### inputs are the name of a startNode and endNode. Given this,
    ### return a list of Nodes that indicates the path from start to finish, using breadth-first search.

    def breadthFirstSearch(self, startNode, endNode):
        been_there = [False] * (max(self.nodeTable) + 1)
        q = queue.Queue()
        been_there[startNode]=True
        node = startNode
        items = ""
        while node != endNode:
            items += str(node) + ", "

            # print("node", node)
            if node not in self.edgeMap:
                node = q.get()
                continue


            edges = self.edgeMap.get(node)

            for edge in edges:
                # print(edge)
                if not been_there[edge.dest]:
                    q.put(edge.dest)
                    been_there[edge.dest] = True
            node = q.get()

        items += str(endNode) + ", "
        print("The BFS", items)
        return items

            ### inputs are the name of a startNode and endNode. Given this,
    ### return a list of Nodes that indicates the path from start to finish, using depth-first search.


    def depthFirstSearch(self, startNode, endNode):
        been_there = [False] * (max(self.nodeTable) + 1)
        stack = deque()
        been_there[startNode]=True
        node = startNode
        items = ""
        while node != endNode:
            items += str(node) + ", "

            # print("node", node)
            if node not in self.edgeMap:
                node = stack.pop()
                continue
            edges = self.edgeMap.get(node)

            for edge in edges:
                # print(edge)
                if not been_there[edge.dest]:
                    stack.append(edge.dest)
                    been_there[edge.dest] = True
            node = stack.pop()

        items += str(endNode) + ", "
        print("The DFS", items)
        return items

    ### implement Djikstra's all-pairs shortest-path algorithm.
    ### https://yourbasic.org/algorithms/graph/#dijkstra-s-algorithm
    ### return the array of distances and the array previous nodes.

    def djikstra(self, startNode):
        distance = [float("inf")] * (max(self.nodeTable) + 1)
        parent = [None] * (max(self.nodeTable) + 1)

        distance[startNode] = 0
        parent[startNode] = 0

        q = queue.Queue()

        for key in self.nodeTable.keys():
            q.put(key)

        while not q.empty():
            current_item = q.get()
            if not self.edgeMap.__contains__(current_item):
                continue
            for neighbour in self.edgeMap.get(current_item):
                if distance[current_item] + abs(neighbour.weight - current_item) < distance[neighbour.dest]:
                    distance[neighbour.dest] =  distance[current_item] + abs(neighbour.weight - current_item)
                    parent[neighbour.dest] = current_item
        print("distance", distance)
        print("parent", parent)
        return distance, parent




    ### takes as input a starting node, and computes the minimum spanning tree, using Prim's algorithm.
    ### https:// en.wikipedia.org/wiki/Prim % 27s_algorithm
    ### you should return a new graph representing the spanning tree generated by Prim's.
    def prim(self, startNode):
        nodes_reached = [startNode]
        nodes_not_reached = list(self.nodeTable.keys())
        nodes_not_reached.remove(startNode)
        prim_tree = []
        while nodes_not_reached:
            # if not self.edgeMap.get(nodes_not_reached): continue
            u = None
            v = None
            temp_weight = float("inf")
            v = float("-inf")
            for aditya in nodes_reached:
                if self.edgeMap.get(aditya) is None:
                    continue
                for selected_edge in self.edgeMap.get(aditya):
                    if selected_edge.weight < temp_weight and selected_edge.dest not in nodes_reached:
                        temp_weight = selected_edge.weight
                        v = selected_edge.dest
                        u = selected_edge.src
            prim_tree.append((u, v))
            v = nodes_not_reached.pop(0)
            nodes_reached.append(v)
        prim_tree.sort()
        print('MST', prim_tree)
        return prim_tree