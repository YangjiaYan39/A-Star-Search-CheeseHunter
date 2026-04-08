from collections import deque
from game_env import GameEnv
from game_state import GameState

import heapq,math
from typing import List,Dict,Tuple
def _get_trap_status(st):
    return getattr(st,'trapStatus', getattr(st,'trap_status'))

class Solver:

    def __init__(self, game_env):
        self.game_env = game_env
        self.maxFrontier = 0
        self.usedNodes = 0
        self.optimalSolution = None
        self.optimalSolutionCost = math.inf
        self.hMemo = {}
        self.nodeId = 0
        self.posmaskBestG = {}

    def transfer(self, trapStatus: List[int]):
        return tuple(trapStatus)
    def _bfsAll(self, startRC):
        R, C = self.game_env.n_rows, self.game_env.n_cols
        grid, SOLID = self.game_env.grid_data, self.game_env.SOLID_TILE
        from collections import deque
        INF = 10 ** 9
        dist = [[INF] * C for _ in range(R)]
        r0, c0 = startRC
        q = deque([(r0, c0)])
        dist[r0][c0] = 0
        while q:
            r, c = q.popleft()
            d = dist[r][c] + 1
            if r > 0 and grid[r - 1][c] != SOLID and dist[r - 1][c] > d:
                dist[r - 1][c] = d
                q.append((r - 1, c))
            if r + 1 < R and grid[r + 1][c] != SOLID and dist[r + 1][c] > d:
                dist[r + 1][c] = d
                q.append((r + 1, c))
            if c > 0 and grid[r][c - 1] != SOLID and dist[r][c - 1] > d:
                dist[r][c - 1] = d
                q.append((r, c - 1))
            if c + 1 < C and grid[r][c + 1] != SOLID and dist[r][c + 1] > d:
                dist[r][c + 1] = d
                q.append((r, c + 1))
        return dist
    def trapStatusToBitmask(self, trapStatus: List[int]):
        bitmask = 0
        for i, s in enumerate(trapStatus):
            if s:
                bitmask |= (1 << i)
        return bitmask
    def _mstCostIdx(self, idxs):
        if not idxs:
            return 0
        mask = 0
        for i in idxs:
            mask |= (1 << i)
        if mask in self.mstCache:
            return self.mstCache[mask]
        used = {idxs[0]}
        total = 0.0
        while len(used) < len(idxs):
            bestD, bestV = math.inf, None
            for v in idxs:
                if v in used:
                    continue
                dmin = min(self.leverDist[u][v] for u in used)
                if dmin < bestD:
                    bestD, bestV = dmin, v
            total += bestD
            used.add(bestV)
        self.mstCache[mask] = total
        return total
    def _isDominated(self, pos, mask, g):
        d = self.posmaskBestG.get(pos)
        if not d:
            return False
        for mPrime, gBest in d.items():
            if (mPrime | mask) == mPrime and gBest <= g:
                return True
        return False
    def _recordPosmask(self, pos, mask, g):
        d = self.posmaskBestG.get(pos)
        if d is None:
            self.posmaskBestG[pos] = {mask: g}
        else:
            best = d.get(mask)
            if best is None or g < best:
                d[mask] = g
    class Search:
        def __init__(self, state, action, parent, cost):
            self.state = state
            self.action = action
            self.parent = parent
            self.cost = cost
        def __lt__(self, other):
            return self.cost < other.cost
        def path(self):
            path = []
            current = self
            while current.parent is not None:
                path.append(current.action)
                current = current.parent
            return path[::-1]
    @staticmethod
    def get_testcases():
        return [1,2,3,4,5,6]

    @staticmethod
    def get_search():
        return "both"

    # === Uniform Cost Search ==========================================================================================
    def search_ucs(self):
        self.startState = self.game_env.get_init_state()
        self.startNode = self.Search(self.startState, None, None, 0)
        frontier = []
        heapq.heapify(frontier)
        startKey=(self.startState.row,self.startState.col,self.trapStatusToBitmask(_get_trap_status(self.startState)))
        usedMap = {startKey: 0}
        heapq.heappush(frontier, (0, self.startNode))
        while frontier:
            cost, node = heapq.heappop(frontier)
            stateKey=(node.state.row,node.state.col,self.trapStatusToBitmask(_get_trap_status(node.state)))
            if node.cost > usedMap.get(stateKey, math.inf):
                continue
            self.usedNodes += 1
            if self.game_env.is_solved(node.state):
                self.reachedCount = len(usedMap)
                self.optimalSolutionCost = node.cost
                self.optimalSolution = node.path()
                return self.optimalSolution
            for action in self.game_env.ACTIONS:
                success, nextState = self.game_env.perform_action(node.state, action)
                if not success:
                    continue
                nextCost = node.cost + self.game_env.ACTION_COST[action]
                nextNode = self.Search(nextState, action, node, nextCost)
                nextKey=(nextState.row,nextState.col,self.trapStatusToBitmask(_get_trap_status(nextState)))
                if nextKey not in usedMap or nextCost < usedMap[nextKey]:
                    usedMap[nextKey] = nextCost
                    heapq.heappush(frontier, (nextCost, nextNode))
            self.maxFrontier = max(self.maxFrontier, len(frontier))
    # === A* Search ====================================================================================================
    def preprocess_heuristic(self):
        """
        Perform pre-processing (e.g. pre-computing repeatedly used values) necessary for your heuristic,
        """
        self.startState = self.game_env.get_init_state()
        self.goal = (self.game_env.goal_row, self.game_env.goal_col)
        self.levers = self.game_env.lever_positions
        self.minStep = min(self.game_env.ACTION_COST.values())
        self.distGoal = self._bfsAll(self.goal)
        self.distLever = [self._bfsAll(lv) for lv in self.levers]
        self.leverGoal = [self.distGoal[r][c] for (r, c) in self.levers]
        nL = len(self.levers)
        self.leverDist = [[math.inf] * nL for _ in range(nL)]
        for i, (ri, ci) in enumerate(self.levers):
            di = self.distLever[i]
            for j, (rj, cj) in enumerate(self.levers):
                self.leverDist[i][j] = 0 if i == j else di[rj][cj]
        self.nL = nL
        self.allMask = (1 << nL) - 1
        self.mstCache = {}
        self.minLeverGoalMask = [0] * (self.allMask + 1)
        for mask in range(self.allMask + 1):
            if mask == 0:
                self.mstCache[mask] = 0
                self.minLeverGoalMask[mask] = 0
                continue
            idxs = [i for i in range(nL) if (mask >> i) & 1]
            self.mstCache[mask] = self._mstCostIdx(idxs)
            self.minLeverGoalMask[mask] = min(self.leverGoal[i] for i in idxs)
    def compute_heuristic(self, state):
        """
        Compute a heuristic value h(n) for the given state.
        :param state: given state (GameState object)
        :return a real number h(n)
        """
        key = (state.row, state.col, self.trapStatusToBitmask(_get_trap_status(state)))
        if key in self.hMemo:
            return self.hMemo[key]
        ts = _get_trap_status(state)
        actMask = self.trapStatusToBitmask(ts)
        remMask = (~actMask) & self.allMask
        if remMask == 0:
            h = self.distGoal[state.row][state.col] * self.minStep
            self.hMemo[key] = h
            return h
        r, c = state.row, state.col
        m = remMask
        dCurLever = math.inf
        while m:
            low = m & -m
            i = low.bit_length() - 1
            d = self.distLever[i][r][c]
            if d < dCurLever:
                dCurLever = d
            m ^= low
        mst = self.mstCache[remMask]
        dLeverGoal = self.minLeverGoalMask[remMask]
        h = (dCurLever + mst + dLeverGoal) * self.minStep
        self.hMemo[key] = h
        return h
    def search_a_star(self):
        self.preprocess_heuristic()
        self.nodeId = 0
        self.hMemo.clear()
        self.optimalSolutionCost = math.inf
        self.posmaskBestG.clear()
        startState = self.game_env.get_init_state()
        startNode = self.Search(startState, None, None, 0)
        frontier = []
        h0 = self.compute_heuristic(startState)
        heapq.heappush(frontier, (h0, 0.0, self.nodeId, startNode))
        self.nodeId += 1
        visited = {}
        startKey = (startState.row, startState.col, self.trapStatusToBitmask(_get_trap_status(startState)))
        visited[startKey] = 0.0
        self.usedNodes = 0
        self.maxFrontier = 1
        while frontier:
            f, negG, _, node = heapq.heappop(frontier)
            if self.game_env.is_solved(node.state):
                self.reachedCount = len(visited)
                self.optimalSolutionCost = node.cost
                self.optimalSolution = node.path()
                return self.optimalSolution
            self.usedNodes += 1
            for action in self.game_env.ACTIONS:
                success, nextState = self.game_env.perform_action(node.state, action)
                if not success:
                    continue
                g2 = node.cost + self.game_env.ACTION_COST[action]
                nextMask = self.trapStatusToBitmask(_get_trap_status(nextState))
                pos = (nextState.row, nextState.col)
                nextKey = (pos[0], pos[1], nextMask)
                if nextKey not in visited or g2 < visited[nextKey]:
                    if self._isDominated(pos, nextMask, g2):
                        continue
                    visited[nextKey] = g2
                    h2 = self.compute_heuristic(nextState)
                    if g2 + h2 >= self.optimalSolutionCost:
                        continue
                    self.nodeId += 1
                    heapq.heappush(frontier, (g2 + h2, -g2, self.nodeId, self.Search(nextState, action, node, g2)))
                    self._recordPosmask(pos, nextMask, g2)
            self.maxFrontier = max(self.maxFrontier, len(frontier))
        return None
        

