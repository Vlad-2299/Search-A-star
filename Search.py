import math
import sys
import pygame
import queue
import copy
import timeit
pygame.init()

running = True
window = True
############### WINDOW #######################################
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Rotas Inteligentes de Recolha de Lixo")
icon = pygame.image.load('way.png')
pygame.display.set_icon(icon)

###############################################################
#PriorityQueue Ordering
class Prioritize:
    def __init__(self, priority, item):
        self.priority = priority
        self.item = item

    def __eq__(self, other):
        return self.priority == other.priority

    def __lt__(self, other):
        return self.priority < other.priority

class gridObj():
    cols = 80
    grid = [0 for i in range(cols)]
    row = 80
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    grey = (220, 220, 220)
    yellow = (255, 255, 0)
    width = 800 / cols
    height = 800 / row

###############################################################
class SearchState(object):
    def __init__(self, coord, parent=None, depth=0, cost=1):
        # if coord[0] < 0 or coord[1] < 0:
        #     raise Exception("The coordinates are not correct!!!")
        self.coord = coord
        self.i = coord[0]
        self.j = coord[1]
        self.parent = parent
        self.depth = depth
        self.cost = cost
        self.f = 0
        self.h = 0
        self.g = 0
        self.neighbors = []
        self.cols = 80
        self.row = 80
        self.obs = False
        self.dest = False
        # self.closed = False

    def show(self, color, st):
        # if self.closed is False:
        pygame.draw.rect(screen, color, (self.i * gridObj.width, self.j * gridObj.height, gridObj.width, gridObj.height), st)
        pygame.display.update()

    # def path(self, color, st):
    #     pygame.draw.rect(screen, color, (self.i * w, self.j * h, w, h), st)
    #     pygame.display.update()

    def addNeighbors(self, grid):
        i = self.i
        j = self.j
        if i < self.cols-1 and grid[self.i + 1][j].obs is False:
            self.neighbors.append(grid[self.i + 1][j])
            if grid[self.i + 1][j].parent is None:
                grid[self.i + 1][j].cost = self.cost + 1
                grid[self.i + 1][j].parent = self
        if i > 0 and grid[self.i - 1][j].obs is False:
            self.neighbors.append(grid[self.i - 1][j])
            if grid[self.i - 1][j].parent is None:
                grid[self.i - 1][j].cost = self.cost + 1
                grid[self.i - 1][j].parent = self
        if j < self.row-1 and grid[self.i][j + 1].obs is False:
            self.neighbors.append(grid[self.i][j + 1])
            if grid[self.i][j + 1].parent is None:
                grid[self.i][j + 1].cost = self.cost + 1
                grid[self.i][j + 1].parent = self
        if j > 0 and grid[self.i][j - 1].obs is False:
            self.neighbors.append(grid[self.i][j - 1])
            if grid[self.i][j - 1].parent is None:
                grid[self.i][j - 1].cost = self.cost + 1
                grid[self.i][j - 1].parent = self

# create 2d array
for i in range(gridObj.cols):
    gridObj.grid[i] = [0 for i in range(gridObj.row)]

# Create Spots
for i in range(gridObj.cols):
    for j in range(gridObj.row):
        gridObj.grid[i][j] = SearchState((i, j), None, 0, 1)

for i in range(gridObj.cols):
    for j in range(gridObj.row):
        gridObj.grid[i][j].show((255, 255, 255), 1)

for i in range(0, gridObj.row):
    gridObj.grid[0][i].show(gridObj.grey, 0)
    gridObj.grid[0][i].obs = True
    gridObj.grid[gridObj.cols-1][i].obs = True
    gridObj.grid[gridObj.cols-1][i].show(gridObj.grey, 0)
    gridObj.grid[i][gridObj.row-1].show(gridObj.grey, 0)
    gridObj.grid[i][0].show(gridObj.grey, 0)
    gridObj.grid[i][0].obs = True
    gridObj.grid[i][gridObj.row-1].obs = True


class PathFinder():
    CONST_START = (20, 20)
    path_cost = 0
    n_visit = 0
    nodesExpanded = 0
    notVisitedSet = set()
    nextToVisitSet = set()
    nextVisitCoord = set()
    notVisitedSet_conf = set()
    nextToVisitSet_conf = set()
    nextVisitCoord_conf = set()
    start = gridObj.grid[20][20]
    end = 0
    start.show(gridObj.blue, 0)

    @staticmethod
    def heuristic(n, e):
        d = math.sqrt((n.i - e.i)**2 + (n.j - e.j)**2)
        # d = abs(n.i - e.i) + abs(n.j - e.j)
        return d

    @classmethod
    def addWall(cls, x):
        t = x[0]
        w = x[1]
        g1 = t // (800 // gridObj.cols)
        g2 = w // (800 // gridObj.row)
        wall = gridObj.grid[g1][g2]
        if wall != cls.start and wall != cls.end:
            if wall.obs is False:
                wall.obs = True

                wall.show((255, 255, 255), 0)

    # @classmethod
    # def clear(cls, x):
    #     t = x[0]
    #     w = x[1]
    #     g1 = t // (800 // gridObj.cols)
    #     g2 = w // (800 // gridObj.row)
    #     target = gridObj.grid[g1][g2]
    #     if target.obs is True:
    #         target.obs = False
    #         target.show((255, 255, 255), 1)

    @classmethod
    def addDest(cls, x):
        t = x[0]
        w = x[1]
        g1 = t // (800 // gridObj.cols)
        g2 = w // (800 // gridObj.row)
        dest = gridObj.grid[g1][g2]
        if dest != cls.start and dest.obs is False:
            # cls.notVisitedSet.add(dest)
            cls.notVisitedSet_conf.add(dest.coord)
            dest.show(gridObj.yellow, 0)

    @classmethod
    def writeOutput(cls, time, found):
        with open('output.txt', 'w') as w:
            if found:
                w.write('Nº de contentores a coletar neste trajeto: ' + str(cls.n_visit) + '\n')
                w.write('Custo do trajeto: ' + str(cls.path_cost) + '\n')
                w.write('Nós processados: ' + str(cls.nodesExpanded) + '\n')
                w.write('Tempo de processamento: ' + str(time) + '\n')
            else:
                w.write('Não foi possível calcular o trajeto selecionado.\nPor favor volte a tentar novamente\n')
        w.close()



    @classmethod
    def pickClosestDest(cls, start, nextVisit):
        prioQueue = queue.PriorityQueue()
        auxNextToVisitSet = nextVisit.copy()
        run = True
        while len(cls.nextToVisitSet_conf) > 0:
            testState = auxNextToVisitSet.pop()
            path = cls.A_star_search(start, testState)
            prioQueue.put(Prioritize(path.cost, path))

        cls.nextToVisitSet.clear()
        cls.nextToVisitSet_conf.clear()

        return prioQueue.get().item

    @classmethod
    def main(cls):
        run = True
        cls.n_visit = len(cls.notVisitedSet_conf)
        start = timeit.default_timer()
        while run:
            cls.nextToVisitSet_conf = cls.notVisitedSet_conf.copy()
            for state in cls.notVisitedSet_conf:
                xCoor = state[0]
                yCoord = state[1]
                cls.nextToVisitSet.add(gridObj.grid[xCoor][yCoord])

            nextState = cls.pickClosestDest(cls.start, cls.nextToVisitSet)
            cls.path_cost += nextState.cost
            cls.printPath(nextState)
            cls.start = nextState

            # Reset Spots
            for i in range(gridObj.cols):
                for j in range(gridObj.row):
                    gridObj.grid[i][j].cost = 1
                    gridObj.grid[i][j].neighbors.clear()
                    gridObj.grid[i][j].parent = None

            cls.notVisitedSet_conf.remove(nextState.coord)
            if len(cls.notVisitedSet_conf) == 0:
                pathHome = cls.A_star_search(nextState, gridObj.grid[20][20])
                cls.path_cost += pathHome.cost
                gridObj.red = (179, 0, 0)
                cls.printPath(pathHome)
                run = False
        stop = timeit.default_timer()
        timeDiff = stop - start
        cls.writeOutput(timeDiff, True)


    @classmethod
    def printPath(cls, state):
        while state.parent is not cls.start:
            state = state.parent
            state.show(gridObj.red, 0)


    @classmethod
    def A_star_search(cls, ini_state, fin_state):
        frontier = queue.PriorityQueue()
        frontier.put(Prioritize(cls.heuristic(ini_state, fin_state), ini_state))
        explored = set()
        frontier_config_set = set()
        frontier_config_set.add(ini_state.coord)

        while not frontier.empty():
            state = frontier.get().item
            frontier_config_set.remove(state.coord)

            if state.coord == fin_state.coord:
                cls.nodesExpanded += len(explored)
                # print('found ' + str(state.cost))
                if state.coord != cls.CONST_START:
                    cls.nextToVisitSet_conf.remove(state.coord)
                return state

            explored.add(state.coord)
            state.addNeighbors(gridObj.grid)
            neighbors = state.neighbors
            for iter in range(len(neighbors)):
                neighbor = neighbors[iter]
                if neighbor.coord not in explored:
                    if neighbor.coord not in frontier_config_set:
                        neighbor.g = state.g + state.cost
                        neighbor.f = neighbor.g + cls.heuristic(neighbor, fin_state)
                        frontier.put(Prioritize(neighbor.f, neighbor))
                        frontier_config_set.add(neighbor.coord)

        cls.writeOutput(None, False)
        sys.exit()

# grid loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 3:
                try:
                    pos = pygame.mouse.get_pos()
                    PathFinder.addDest(pos)
                except AttributeError:
                    pass

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:

                # PathFinder.end = next(iter(PathFinder.nextDest))
                # PathFinder.frontier.put(Prioritize(PathFinder.heuristic(PathFinder.start, PathFinder.end), PathFinder.start))
                # PathFinder.frontier_set.add(PathFinder.start)
                running = False
                break

        if pygame.mouse.get_pressed()[0]:
            try:
                pos = pygame.mouse.get_pos()
                PathFinder.addWall(pos)
            except AttributeError:
                pass
while window:
    ev = pygame.event.poll()
    if ev.type == pygame.QUIT:
        window = False
    pygame.display.update()
    PathFinder.main()
