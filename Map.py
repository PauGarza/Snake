import random

# Direcciones:

UP = (0, 1)
DOWN = (0, -1)
RIGHT = (1, 0)
LEFT = (-1, 0)

ALL_DIR = (UP, DOWN, RIGHT, LEFT)

HORZ = [RIGHT, LEFT]  # Direcciones horizontales
VER = [UP, DOWN]  # Direcciones verticales


# Clase que representa un nodo en un grafo utilizado en el proyecto
class Node:
    # Lista de direcciones que no se pueden alcanzar
    BlockingDireccions = []

    # Atributo booleano que indica si el nodo ha sido visitado
    Visited: bool = False

    # Constructor de la clase Node, se utiliza para inicializar los atributos de instancia
    def __init__(self) -> None:
        # Establece el valor de "Visited" en "False" para indicar que el nodo no ha sido visitado todavía
        self.Visited = False

        # Inicializa una lista vacía en "BlockingDireccions"
        self.BlockingDireccions = []


# Función que calcula la distancia simple entre dos puntos
def CalculateSimpleDistance(P1: tuple[int, int], P2: tuple[int, int]):
    # Calcula la diferencia en el eje X y Y entre los dos puntos y los suma para obtener la distancia simple
    return abs(P1[0] - P2[0]) + abs(P1[1] - P2[1])


# Función que mueve un punto en una dirección determinada
def Mover(Point: tuple[int, int], Dir: tuple[int, int]):
    # Suma el desplazamiento en X y Y de la dirección a las coordenadas del punto para obtener su nueva posición
    return (Point[0] + Dir[0], Point[1] - Dir[1])


# Función que se utiliza para comparar elementos en una lista
def Compare(l):
    # Devuelve el segundo elemento de la lista, que se utiliza para comparar
    return l[1]


# Función principal que utiliza el algoritmo A* para encontrar el camino más corto desde "Inicio" hasta "Target" en un mapa dado
def AStarPathFinding(Map: list, Inicio: int, Target: int, limit: int = None):
    # Lista que almacena los valores de distancia y dirección para cada vértice
    Vertices = [[None, None, None, None]] * len(Map)
    # [Distancia desde el principio, distancia al objetivo, la dirección de la que venimos, el índice del vértice del que venimos]

    l = 0  # Contador de iteraciones para prevenir ciclos infinitos
    path = []  # Lista que almacenará el camino más corto desde "Inicio" hasta "Target"
    # Lista que almacena los vértices a explorar y su distancia desde el origen
    v = [(Map[Inicio], 0)]

    # Establece los valores iniciales de distancia para el vértice de inicio en la lista "Vertices"
    Vertices[Inicio] = [0, 0, None, None]

    Trg = Map[Target]  # Las coordenadas objetivo
    while len(v) > 0:
        if limit != None:
            l += 1
            if l >= limit:
                raise Exception("Overflow")
        cv = v.pop(0)  # Vértice actual
        cindex = Map.index(cv[0])  # Índice del vértice actual en el mapa
        if cindex != Target:  # Si el vértice actual no es el objetivo
            for dir in ALL_DIR:  # Recorre todas las direcciones posibles desde el vértice actual
                # Calcula el nuevo punto de acuerdo a la dirección
                NP = Mover(cv[0], dir)
                if NP in Map:  # Si el nuevo punto está en el mapa
                    # Obtiene el índice del nuevo vértice en el mapa
                    index = Map.index(NP)
                    # Calcula la distancia desde el origen al nuevo vértice a través del vértice actual
                    Dis1 = Vertices[cindex][0] + \
                        CalculateSimpleDistance(NP, cv[0])
                    # Calcula la distancia desde el nuevo vértice al objetivo
                    Dis2 = CalculateSimpleDistance(
                        NP, Trg) - (1 if dir == Vertices[cindex][3] else 0)
                    # Obtiene los valores actuales de distancia y dirección del vecino actual
                    CNeighbor = Vertices[index]
                    # Si el vecino actual aún no ha sido visitado o la nueva distancia es menor que la actual
                    # y la distancia desde el objetivo es menor que la distancia actual desde el origen del vecino objetivo
                    if (CNeighbor[0] == None or Dis2 < CNeighbor[1]) and (Vertices[Target][0] == None or Vertices[Target][0] > Dis2):
                        # Agrega el nuevo vértice a la lista de vértices por explorar
                        v.append((NP, Dis2))
                        # Actualiza los valores de distancia y dirección del vecino actual
                        Vertices[index] = [Dis1, Dis2, dir, cindex]
            # Ordena la lista de vértices por explorar por distancia al objetivo
            v.sort(key=Compare)
        else:  # Si el vértice actual es el objetivo
            break  # Termina el bucle

    if Vertices[Target][0] == None:  # Si no se encontró una solución
        raise Exception("No se encontró tal camino al objetivo")
    else:
        cv = Vertices[Target]  # Vértice actual
        while cv != Vertices[Inicio]:  # Mientras el vértice actual no sea el inicio
            # Agrega la dirección del vértice actual al camino más corto
            path.append(cv[2])
            # Actualiza el vértice actual al vértice del que venimos
            cv = Vertices[cv[3]]

        path.reverse()  # Invierte el orden del camino para que vaya desde el inicio hasta el objetivo
        return path  # Devuelve el camino más corto desde el inicio hasta el objetivo


def GenerateEmptiesMap(snk, apl, Gx: int, Gy: int, ExceptHead=False, Exceptapl=False):
    """
    Devuelve todas las coordenadas de puntos vacíos en un juego de serpiente, es decir, puntos que no incluyen el cuerpo de la serpiente y, opcionalmente, la manzana.

    :param snk: el cuerpo de la serpiente.
    :param apl: la manzana o su posición.
    :param Gx: el ancho del mapa.
    :param Gy: la altura del mapa.
    :param ExceptHead: configúralo en verdadero si deseas considerar la posición de la cabeza como vacía.
    :param Exceptapl: configúralo en verdadero si deseas excluir la posición de la manzana.

    :return: lista que contiene todas las coordenadas de puntos vacíos.
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
    """
    Genera una lista de todas las posiciones del mapa.

    :param Gx: el ancho del mapa.
    :param Gy: la altura del mapa.
    :param reverse: configúralo en verdadero si deseas generar el mapa de derecha a izquierda.

    :return: lista que contiene todas las posiciones del mapa.
    """
    toreturn = []  # Lista que almacenará todas las posiciones del mapa

    for y in range(Gy):
        for x in range(Gx):
            if reverse:
                # Si se desea revertir la generación, se agrega la posición (y, x) a la lista
                toreturn.append((y, x))
            else:
                # De lo contrario, se agrega la posición (x, y) a la lista
                toreturn.append((x, y))

    return toreturn  # Devuelve la lista de todas las posiciones del mapa


# Función que genera un grafo a partir de un mapa
def GenerateMapGraph(Mp: list):
    # Lista de adyacencia para almacenar el grafo generado
    Graph = [None] * len(Mp)

    for i in range(len(Mp)):
        # Obtiene el punto en la posición del índice i en el mapa
        Point = Mp[i]

        for dir in ALL_DIR:
            # El nuevo punto si seguimos esa dirección
            NP = (Point[0] + dir[0], Point[1] - dir[1])

            if NP in Mp:  # Si el nuevo punto está en el mapa
                if Graph[i] == None:
                    # Crea una nueva lista de adyacencia para el punto si aún no existe
                    Graph[i] = []

                # Agrega la posición del nuevo punto al grafo como un nuevo vértice adyacente con una distancia de 1
                Graph[i].append([Mp.index(NP), 1])

    return Graph  # Devuelve el grafo generado como una lista de adyacencia


# Función que implementa el algoritmo de Prim para encontrar el árbol de expansión mínima de un grafo
def PrimsAlgorithm(G: list, Inicio: int):

    visited = [Inicio]  # Lista de vértices visitados
    sol = [None] * len(G)  # Lista que almacenará la solución del algoritmo

    while len(visited) < len(G):
        # La distancia más pequeña (la longitud / peso del borde más pequeña)
        SmallestDis = None
        ver = None  # Vértice actual
        ver2 = None  # Vértice adyacente
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
                    # Elimina el borde para que no se busque nuevamente (para mejorar la eficiencia)
                    G[v].remove(edge)

        if len(rv) > 0:
            i = random.randint(0, len(rv)-1)
            ver = rv[i]
            ver2 = re[i][0]
            SmallestDis = re[i][1]

        G[ver].remove([ver2, SmallestDis])
        G[ver2].remove([ver, SmallestDis])
        # Elimina los bordes para que no se busquen nuevamente (para mejorar la eficiencia)

        if sol[ver] == None:
            sol[ver] = []

        sol[ver].append([ver2, SmallestDis])
        # Agrega el vértice adyacente a la lista de vértices visitados
        visited.append(ver2)

    return sol  # Devuelve la solución del algoritmo como una lista de adyacencia

# Función que genera un laberinto hamiltoniano a partir de un grafo y un mapa


def GenerateHamiltonianMaze(Graph: list, Map: list, Gx: int, Gy: int) -> list[Node]:
    l = Gx * Gy
    Maze = [None] * l

    for i in range(l):
        Maze[i] = Node()  # Crea un nuevo nodo para cada posición del laberinto

    for i in range(len(Graph)):
        if Graph[i] != None:
            vx, vy = Map[i]  # Las coordenadas X e Y del vértice en el mapa

            for edge in Graph[i]:
                nvx = vx * 2 + 1  # La coordenada X proyectada en el laberinto
                nvy = vy * 2 + 1  # La coordenada Y proyectada en el laberinto

                # Las coordenadas del vértice que está adyacente al vértice i
                vx2, vy2 = Map[edge[0]]

                # La dirección del vértice a su adyacente (vértice2)
                Dir = (vx2 - vx, vy - vy2)

                nvx2 = vx2 * 2 + 1
                nvy2 = vy2 * 2 + 1

                if nvx2 < nvx or nvy2 < nvy:
                    nvx = nvx2
                    nvy = nvy2
                    # Invierte la dirección si es necesario
                    Dir = (-Dir[0], -Dir[1])

                for j in range(2):
                    if Dir in HORZ:
                        # Bloquea el camino en ambos lados si la dirección es horizontal
                        Maze[Gx * nvy + nvx].BlockingDireccions.append(UP)
                        Maze[Gx * (nvy-1) +
                             nvx].BlockingDireccions.append(DOWN)

                        nvx += Dir[0]
                    else:
                        # Bloquea el camino en ambos lados si la dirección es vertical
                        Maze[Gx * nvy + nvx].BlockingDireccions.append(LEFT)
                        Maze[Gx * nvy + nvx -
                             1].BlockingDireccions.append(RIGHT)

                        nvy -= Dir[1]

    return Maze  # Devuelve el laberinto hamiltoniano generado como una lista de nodos


# Función que verifica si una posición (x, y) está dentro de los límites del mapa con dimensiones (Gx, Gy)
def InsideBorders(x: int, y: int, Gx: int, Gy: int):
    # Verifica si x está dentro de los límites horizontales del mapa
    # y si y está dentro de los límites verticales del mapa
    return x < Gx and x >= 0 and y < Gy and y >= 0


# Función que convierte un laberinto en un ciclo hamiltoniano
def Maze2Cycle(Maze: list[Node], Gx: int, Gy: int):
    """
    Esta función, llamada `Maze2Cycle`, utiliza el algoritmo de Prim para generar un árbol de expansión mínimo en forma de laberinto. Luego, convierte este laberinto en un ciclo hamiltoniano utilizando la técnica de "caminar sobre el borde".
    La función toma como entrada la lista de nodos del laberinto (`Maze`) y las dimensiones del mapa (`Gx` y `Gy`). Devuelve dos valores: la lista `sol`, que contiene todas las posiciones del ciclo hamiltoniano, y la lista `path`, que contiene las direcciones de movimiento necesarias para seguir el ciclo hamiltoniano.
    El algoritmo de Prim se utiliza para construir un árbol de expansión mínimo en forma de laberinto a partir de un grafo no dirigido ponderado. El laberinto resultante se representa como una lista de nodos (`Maze`), donde cada nodo contiene una lista de las direcciones en las que se pueden mover desde ese nodo.
    Para convertir el laberinto en un ciclo hamiltoniano, la función utiliza la técnica de "caminar sobre el borde", que implica seguir las paredes del laberinto y volver sobre sus pasos cuando sea necesario. En cada paso, se comprueba si la dirección actual está bloqueada por una pared (`BlockingDireccions` en el nodo actual). Si la dirección actual no está bloqueada y no lleva a un nodo visitado anteriormente, se avanza en esa dirección. De lo contrario, se busca una dirección no bloqueada que no conduzca a un nodo visitado. La función sigue este proceso hasta que se completa el ciclo hamiltoniano.
    """
    P = (0, 0)  # Posición actual
    dir = (1, 0)  # la dirección que vamos
    # Nuestra dirección correcta en relación con nuestra dirección orientada
    right = (0, -1)

    l = Gx * Gy

    sol = [P]  # la solución que hemos encontrado
    path = []  # el camino del ciclo hamiltoniano a seguir

    for N in range(l):
        x, y = P
        # Verifica si la dirección actual no está bloqueada y si la siguiente posición está dentro de los límites del mapa
        if dir not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(x + dir[0], y - dir[1], Gx, Gy):
            Maze[y * Gx + x].Visited = True

            # Actualiza la posición y agrega a la solución
            x += dir[0]
            y -= dir[1]
            P = (x, y)
            sol.append(P)
            path.append(dir)
        else:
            # Busca una dirección no bloqueada que no conduce a un nodo visitado
            for d in ALL_DIR:
                nx = x + d[0]
                ny = y - d[1]
                NP = (nx, ny)

                # Verifica si la dirección no está bloqueada y si la siguiente posición está dentro de los límites del mapa y no ha sido visitada
                if d not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(nx, ny, Gx, Gy) and not Maze[ny * Gx + nx].Visited:
                    Maze[y * Gx + x].Visited = True
                    P = NP
                    dir = d

                    if dir in HORZ:  # Nuestra dirección es horizontal
                        right = (0, -dir[0])
                    else:  # Nuestra dirección es vertical
                        right = (dir[1], 0)

                    # Agrega la nueva posición y dirección a las soluciones
                    sol.append(P)
                    path.append(dir)
                    break

        # Verifica si podemos ir hacia la dirección derecha
        if right not in Maze[y * Gx + x].BlockingDireccions and InsideBorders(x + right[0], y - right[1], Gx, Gy):
            dir = right

            if dir in HORZ:  # Nuestra dirección es horizontal
                right = (0, -dir[0])
            else:  # Nuestra dirección es vertical
                right = (dir[1], 0)

            # Agrega la dirección hacia la derecha al camino del ciclo hamiltoniano
    return sol, path
