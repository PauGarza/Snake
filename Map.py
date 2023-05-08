import random

# Direcciones:

UP = (0, 1)
DOWN = (0, -1)
RIGHT = (1, 0)
LEFT = (-1, 0)

ALL_DIR = (UP, DOWN, RIGHT, LEFT)

HORZ = [RIGHT, LEFT]  # Direcciones horizontales
VER = [UP, DOWN]  # Direcciones verticales


class Node:
    BlockingDireccions = []  # las direcciones que no podemos alcanzar

    Visited: bool = False

    def __init__(self) -> None:
        self.Visited = False
        self.BlockingDireccions = []


def CalculateSimpleDistance(P1: tuple[int, int], P2: tuple[int, int]):
    return abs(P1[0] - P2[0]) + abs(P1[1] - P2[1])


def Mover(Point: tuple[int, int], Dir: tuple[int, int]):
    return (Point[0] + Dir[0], Point[1] - Dir[1])


def Compare(l):
    return l[1]


def AStarPathFinding(Map: list, Inicio: int, Target: int, limit: int = None):
    Vertices = [[None, None, None, None]] * len(Map)
    # [Distancia desde el principio, distancia al objetivo, la dirección de la que venimos, el índice del vértice del que venimos]
    l = 0

    path = []

    v = [(Map[Inicio], 0)]

    Vertices[Inicio] = [0, 0, None, None]

    Trg = Map[Target]  # Las coordenadas objetivo
    while len(v) > 0:
        if limit != None:
            l += 1
            if l >= limit:
                raise Exception("Overflow")
        cv = v.pop(0)  # Vértice actual
        cindex = Map.index(cv[0])
        if cindex != Target:
            for dir in ALL_DIR:
                NP = Mover(cv[0], dir)
                if NP in Map:
                    index = Map.index(NP)
                    Dis1 = Vertices[cindex][0] + \
                        CalculateSimpleDistance(NP, cv[0])
                    Dis2 = CalculateSimpleDistance(
                        NP, Trg) - (1 if dir == Vertices[cindex][3] else 0)
                    CNeighbor = Vertices[index]
                    if (CNeighbor[0] == None or Dis2 < CNeighbor[1]) and (Vertices[Target][0] == None or Vertices[Target][0] > Dis2):
                        v.append((NP, Dis2))
                        Vertices[index] = [Dis1, Dis2, dir, cindex]
            v.sort(key=Compare)
        else:
            break
    if Vertices[Target][0] == None:  # No encontramos una solución
        raise Exception("No se encontró tal camino al objetivo")
    else:
        cv = Vertices[Target]  # Vértice actual
        while cv != Vertices[Inicio]:
            path.append(cv[2])
            cv = Vertices[cv[3]]

        path.reverse()
        return path


def GenerateEmptiesMap(snk, apl, Gx: int, Gy: int, ExceptHead=False, Exceptapl=False):
    """Devuelve todas las coordenadas de puntos vacíos en un juego de serpiente: puntos que no incluyen el cuerpo de la serpiente (opcionalmente la manzana) \ n
    : parámetro: \ n
    SNK: el cuerpo de la serpiente \ n
    Apple: la manzana o su posición \ n
    Eliminar: posiciones para eliminar del mapa \ n \ n

    ExcepThead: configúrelo en verdadero si desea considerar la posición de la cabeza como vacía \ n \ n

    Gx: el ancho del mapa \ n
    Gy: la altura del mapa
    """

    # Posiciones del cuerpo de Snake
    SPOS = snk.Pos if not ExceptHead else snk.Pos[1:len(snk)]

    APOS = apl.Pos  # manzanaPosition

    toreturn = []
    for y in range(Gy):
        for x in range(Gx):
            if (x, y) not in SPOS:
                if apl == None or (x, y) != APOS or Exceptapl:
                    toreturn.append((x, y))

    return toreturn


def GenerateMap(Gx: int, Gy: int, reverse: bool = False):
    """Genera una lista de todas las posiciones del mapa
        GX: El ancho del mapa
        Gy: la altura del mapa
        Reversa: generalmente el mapa se generará a partir de la esquina izquierda hacia abajo.
                    Reversar significa que comenzaremos en la misma posición yendo a la derecha
    """

    toreturn = []

    for y in range(Gy):
        for x in range(Gx):
            if reverse:
                toreturn.append((y, x))
            else:
                toreturn.append((x, y))

    return toreturn


def GenerateMapGraph(Mp: list):

    Graph = [None] * len(Mp)

    for i in range(len(Mp)):
        Point = Mp[i]  # el punto del índice I en el mapa

        for dir in ALL_DIR:
            # El nuevo punto si seguimos esa dirección
            NP = (Point[0] + dir[0], Point[1] - dir[1])

            if NP in Mp:
                if Graph[i] == None:
                    Graph[i] = []

                Graph[i].append([Mp.index(NP), 1])

    return Graph


def PrimsAlgorithm(G: list, Inicio: int):

    visited = [Inicio]

    sol = [None] * len(G)  # la solución

    while len(visited) < len(G):
        # la distancia más pequeña (la longitud / peso del borde más pequeña)
        SmallestDis = None
        ver = None
        ver2 = None

        rv = []  # Lista de vértices para elegir entre al azar al encontrar dos bordes con la misma longitud
        re = []  # Lista de bordes para elegir entre al azar al encontrar dos bordes con la misma longitud

        for v in visited:
            for edge in G[v]:
                if edge[0] not in visited and (SmallestDis == None or edge[1] < SmallestDis):
                    rv = []
                    re = []
                    SmallestDis = edge[1]
                    ver = v
                    ver2 = edge[0]
                elif edge[0] not in visited and SmallestDis == edge[1]:
                    rv.append(v)
                    re.append(edge)
                elif edge[0] in visited:
                    # Eliminar el borde para que no lo busquemos nuevamente (para mejorar la eficiencia)
                    G[v].remove(edge)

        if len(rv) > 0:
            i = random.randint(0, len(rv)-1)
            ver = rv[i]
            ver2 = re[i][0]
            SmallestDis = re[i][1]

        G[ver].remove([ver2, SmallestDis])
        G[ver2].remove([ver, SmallestDis])
        # Eliminar los bordes para que no los busquemos nuevamente (para mejorar la eficiencia)

        if sol[ver] == None:
            sol[ver] = []

        sol[ver].append([ver2, SmallestDis])

        visited.append(ver2)

    return sol


def GenerateHamiltonianMaze(Graph: list, Map: list, Gx: int, Gy: int) -> list[Node]:
    l = Gx * Gy

    Maze = [None] * l

    for i in range(l):
        Maze[i] = Node()

    for i in range(len(Graph)):
        if Graph[i] != None:
            vx, vy = Map[i]  # Las coordenadas X e Y del vértice

            for edge in Graph[i]:

                nvx = vx * 2 + 1  # la coordenada X proyectada
                nvy = vy * 2 + 1  # la coordenada y proyectada

                # las coordenadas del vértice que está adyacente al gráfico [i] vértice
                vx2, vy2 = Map[edge[0]]

                # La dirección del vértice a su adyacente (vértice2)
                Dir = (vx2 - vx, vy - vy2)

                nvx2 = vx2 * 2 + 1
                nvy2 = vy2 * 2 + 1

                if nvx2 < nvx or nvy2 < nvy:
                    nvx = nvx2
                    nvy = nvy2
                    Dir = (-Dir[0], -Dir[1])

                for j in range(2):
                    if Dir in HORZ:
                        Maze[Gx * nvy + nvx].BlockingDireccions.append(UP)
                        Maze[Gx * (nvy-1) +
                             nvx].BlockingDireccions.append(DOWN)
                        # Bloqueando el camino en ambos lados

                        nvx += Dir[0]
                    else:
                        Maze[Gx * nvy + nvx].BlockingDireccions.append(LEFT)
                        Maze[Gx * nvy + nvx -
                             1].BlockingDireccions.append(RIGHT)
                        # Bloqueando el camino en ambos lados

                        nvy -= Dir[1]

                pass

    return Maze


def InsideBorders(x: int, y: int, Gx: int, Gy: int):
    return x < Gx and x >= 0 and y < Gy and y >= 0


def Maze2Cycle(Maze: list[Node], Gx: int, Gy: int):
    P = (0, 0)  # Posición actual
    dir = (1, 0)  # la dirección que vamos
    # Nuestra dirección correcta en relación con nuestra dirección orientada
    right = (0, -1)
    # Piénselo como tu mano derecha
    # Si se encuentra frente al norte, su dirección de la derecha es el este
    # Pero si estás mirando al sur, tu dirección derecha es Occidente

    l = Gx * Gy

    sol = [P]  # la solución que hemos encontrado

    path = []  # el camino del ciclo hamiltoniano a seguir

    for N in range(l):
        x, y = P
        # Nuestra dirección no está bloqueada
        if dir not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(x + dir[0], y - dir[1], Gx, Gy):
            Maze[y * Gx + x].Visited = True

            x += dir[0]
            y -= dir[1]

            P = (x, y)
            sol.append(P)
            path.append(dir)
        else:
            # Buscaremos una dirección no bloqueada que no conduzca a un nodo visitado
            for d in ALL_DIR:
                nx = x + d[0]
                ny = y - d[1]
                NP = (nx, ny)

                if d not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(nx, ny, Gx, Gy) and not Maze[ny * Gx + nx].Visited:
                    Maze[y * Gx + x].Visited = True
                    P = NP
                    dir = d

                    if dir in HORZ:  # Nuestra dirección es horizontal
                        right = (0, -dir[0])
                    else:  # Nuestra dirección es vertical
                        right = (dir[1], 0)

                    sol.append(P)
                    path.append(dir)
                    break
        # Podemos ir derecha
        if right not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(x + right[0], y - right[1], Gx, Gy):
            dir = right

            if dir in HORZ:  # Nuestra dirección es horizontal
                right = (0, -dir[0])
            else:  # Nuestra dirección es vertical
                right = (dir[1], 0)
            # girando nuestra dirección hacia la derecha

    return sol, path
