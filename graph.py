class Node:
    def __init__(self, x_coord, y_coord, is_free):
        self.x: float = x_coord
        self.y: float = y_coord
        self.free: bool = is_free

class Edge:
    def __init__(self, node_1, node_2):
        self.node_1: Node = node_1
        self.node_2: Node = node_2

class Graph:
   def __init__(self):
        self.nodes: list(Node) = []
        self.edges: list(Edge) = []