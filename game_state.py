class GameState:
    def __init__(self, row, col, trap_status):
        self.row = row
        self.col = col
        assert isinstance(trap_status, tuple), '!!! trap_status should be a tuple !!!'
        self.trap_status = trap_status

    def __eq__(self, other):
        if not isinstance(other, GameState):
            return False
        return self.row == other.row and self.col == other.col and self.trap_status == other.trap_status

    def __hash__(self):
        return hash((self.row, self.col, self.trap_status))

    def __repr__(self):
        return f'row: {self.row},\t\t col: {self.col}, \t\t trap status: {self.trap_status}'

    def deepcopy(self):
        return GameState(self.row, self.col, self.trap_status)
