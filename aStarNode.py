class aStarNode:
    def __init__(self, parent, child, state, functionG, cost, neighbors, move):
        self.parent = parent
        self.child = child
        self.state = state
        self.functionG = functionG
        self.cost = cost
        self.move = move
        self.neighbors = neighbors

    def __eq__(self, other):
        return self.cost == other

    def __lt__(self, other):
        return self.cost < other

    def __gt__(self, other):
        return self.cost > other
