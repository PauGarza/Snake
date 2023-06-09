import pygame as pgm
import sys
import random

import Map

import pygame.time

#   Pantalla y Grid Tam :
tamaño = w, h = 400, 400

Grid_Tam = 40

GX = w // Grid_Tam
GY = h // Grid_Tam
# gx y gy deben ser dos números pares

LEN = GX * GY  # la longitud del mapa (número de posiciones posibles)

#    Colores principales:

BCKG_Color = (106, 128, 87)   # Verde pasto
Manzana_CLR = (255, 51, 51)  # Rojo claro
SNK_CLR = (200, 200, 200)    # Gris claro
TXT_CLR = (255, 255, 255)    # Blanco


# Carga la imagen de fondo
background = pygame.image.load("background.png")

#    Instrucciones de serpiente:

UP = (0, 1)
DOWN = (0, -1)
RIGHT = (1, 0)
LEFT = (-1, 0)

ALL_DIR = (UP, DOWN, RIGHT, LEFT)

HORZ = [RIGHT, LEFT]  # Direcciones horizontales
VER = [UP, DOWN]  # Vertical Direccions


class Snake:
    Length = 1
    Pos = []
    Direccion = UP  # Direccion
    Dead = False
    ColaPos = None  # El último puesto de cola antes del movimiento
    # Se usa para expandir la serpiente en la dirección correcta

    ColaDir = None  # la última dirección que tomó la cola

    # representa el relleno que hacemos cuando dibujamos el cuerpo de la serpiente
    Body_Padding = 2

    def __init__(self, Pos=[(GX // 2, GY // 2)], Direccion=random.choice(ALL_DIR), Body_Padding=2):
        self.Length = 1
        self.Pos = Pos
        self.Length = len(Pos)
        self.Direccion = Direccion
        self.Body_Padding = Body_Padding

    def __len__(self):
        return self.Length

    @staticmethod
    def Draw_Piece(Pos1: tuple | list, Pos2: tuple | list, Pos3: tuple | list, Body_Padding=2, Color: tuple = SNK_CLR):

        DirX = Pos1[0] - Pos2[0]
        DirY = Pos1[1] - Pos2[1]
        Dir = (DirX, DirY)  # direcion nueva

        if Dir == (1, 0):  # Derecha
            left = Pos1[0] * Grid_Tam - Body_Padding
            top = Pos1[1] * Grid_Tam + Body_Padding

            pgm.draw.rect(scr, Color, pgm.Rect(
                left, top, Grid_Tam, Grid_Tam - 2 * Body_Padding))
        elif Dir == (-1, 0):  # Izquierda
            left = Pos1[0] * Grid_Tam + Body_Padding
            top = Pos1[1] * Grid_Tam + Body_Padding

            pgm.draw.rect(scr, Color, pgm.Rect(
                left, top, Grid_Tam, Grid_Tam - 2 * Body_Padding))
        elif Dir == (0, -1):  # Arriba
            left = Pos1[0] * Grid_Tam + Body_Padding
            top = Pos1[1] * Grid_Tam + Body_Padding

            pgm.draw.rect(scr, Color, pgm.Rect(
                left, top, Grid_Tam - 2 * Body_Padding, Grid_Tam))
        elif Dir == (0, 1):  # Abajo
            left = Pos1[0] * Grid_Tam + Body_Padding
            top = Pos1[1] * Grid_Tam - Body_Padding

            pgm.draw.rect(scr, Color, pgm.Rect(
                left, top, Grid_Tam - 2 * Body_Padding, Grid_Tam))

        if Pos3 != None:
            DrawPos3 = True

            NDirX = Pos3[0] - Pos1[0]
            NDirY = Pos3[1] - Pos1[1]
            NDir = (NDirX, NDirY)

            if NDir == Dir:  # Si las piezas tienen la misma dirección
                DrawPos3 = False

            if DrawPos3:
                Snake.Draw_Piece(Pos3, Pos1, None, Body_Padding, Color)

    def Draw(self, scr, Color: tuple = SNK_CLR):

        if len(self.Pos) == 1:
            for i in self.Pos:
                left = i[0] * Grid_Tam + self.Body_Padding
                top = i[1] * Grid_Tam + self.Body_Padding

                pgm.draw.rect(scr, Color, pgm.Rect(
                    left, top, Grid_Tam - 2 * self.Body_Padding, Grid_Tam - 2 * self.Body_Padding))
        else:
            for i in range(len(self.Pos) - 1):  # Dibujando cada pieza excepto la cola
                Snake.Draw_Piece(self.Pos[i], self.Pos[i + 1], None if i ==
                                 0 else self.Pos[i - 1],  self.Body_Padding, Color)

            Snake.Draw_Piece(self.Pos[self.Length - 1], self.Pos[self.Length - 2],
                             None, self.Body_Padding, Color)  # drawing the tail

    def Move(self, reverse=False, DontEditLT: bool = False):
        head = self.Pos[0]
        if reverse:
            p = (head[0] - self.Direccion[0], head[1] + self.Direccion[1])
        else:
            p = (head[0] + self.Direccion[0], head[1] - self.Direccion[1])
        self.Pos.insert(0, p)
        if not DontEditLT:
            self.ColaPos = self.Pos.pop(len(self.Pos) - 1)
            tail = self.Pos[len(self) - 1]
            self.ColaDir = (tail[0] - self.ColaPos[0],
                            self.ColaPos[1] - tail[1])
        else:
            self.Pos.pop()

    def Change_Direccion(self, Dir):
        Dir = tuple(Dir)
        self.Direccion = Dir

    def Append(self):
        self.Pos.append(self.ColaPos)
        self.Length += 1

    def ver_si_dead(self):
        "" "Devuelve verdadero si la serpiente está en una posición de muerte" ""

        head = self.Pos[0]

        HX = head[0]  # Coordenada de cabeza x

        HY = head[1]  # Coordinada de Head Y

        if HX < 0 or HX >= GX or HY < 0 or HY >= GY or head in self.Pos[1:]:
            return True


class Manzana:

    Pos = (0, 0)

    scr = None  # pantalla ||superficie para dibujar la manzana

    def __init__(self, scr, SNK: Snake, Pos: tuple = None):
        self.scr = scr

        self.Pos = Pos
        if Pos == None:
            self.Respawn(SNK)

    def Can_Eat(self, snk: Snake, Respawn=True):
        if snk.Pos[0] == self.Pos:
            snk.Append()
            if Respawn:
                self.Respawn(snk)

    def Respawn(self, snk: Snake | tuple[int]):
        """Respawn la manzana (se usa si se ha comido la manzana)"""

        # Obtener todos los espacios vacíos
        emp = Map.GenerateEmptiesMap(snk, self, GX, GY)
        # sus para asegurarse de que la manzana no se reptee en el mismo lugar o en el cuerpo de la serpiente

        if len(emp) > 0:
            # Aleatorizando la posición de la manzana
            self.Pos = random.choice(emp)

    def Draw(self):
        """Dibuja la Manzana en el Pantalla"""

        left = self.Pos[0] * Grid_Tam
        top = self.Pos[1] * Grid_Tam

        pygame.draw.rect(self.scr, Manzana_CLR, pygame.Rect(
            left, top, Grid_Tam, Grid_Tam), border_radius=8)


def Mover(Point: tuple[int, int], Dir: tuple[int, int], reverse=False):
    return ((Point[0] - Dir[0], Point[1] + Dir[1]) if reverse else (Point[0] + Dir[0], Point[1] - Dir[1]))


def PosibleDir(snk: Snake, Dir: tuple, reverse=False):

    # Pruebas si una dirección es segura para seguir

    head = snk.Pos[0]

    head = Mover(head, Dir, reverse)

    return (not head in snk.Pos) and Map.InsideBorders(head[0], head[1], GX, GY)


def PuedeContinuarHamiltonianCirculo(snk: Snake, HamiltonianCirculo: list, Gx: int = GX, Gy: int = GY):
    test1 = True
    test2 = True

    reversehc = False

    l = len(snk)

    Len = Gx * Gy

    Mp = [None] * Len

    for i in range(l):
        # un pedazo del cuerpo de serpiente en un índice específico
        pieza = snk.Pos[l - i - 1]
        index = pieza[1] * Gx + pieza[0]  # el índice de la pieza
        Mp[index] = i + 1

    # el índice de cola cuando la serpiente sigue el HamiltonianCirculo
    s = (HamiltonianCirculo.index(
        snk.Pos[0]) + 1) % (len(HamiltonianCirculo) - 1)
    saveds = s
    # El índice de la cabeza cuando la serpiente sigue al Hamiltoniancircolo
    e = (s + l - 1) % (len(HamiltonianCirculo) - 1)

    i = 1

    while s != e:  # Prueba el Hamiltoniancirculo en su dirección normal
        pieza = HamiltonianCirculo[s]
        index = pieza[1] * Gx + pieza[0]
        if Mp[index] != None and i <= Mp[index]:
            test1 = False
            break
        s = (s+1) % (len(HamiltonianCirculo) - 1)
        i += 1

    if not test1:
        reversehc = True

        s = (saveds - 1) % (len(HamiltonianCirculo) - 1)
        s = s if s > 0 else len(HamiltonianCirculo) - 1 + s
        e = (s - l + 1) % (len(HamiltonianCirculo) - 1)
        e = e if e > 0 else len(HamiltonianCirculo) - 1 + e

        i = 0

        while e != s:  # prueba el HamiltonianCirculo en su dirección invertida
            pieza = HamiltonianCirculo[e]
            index = pieza[1] * Gx + pieza[0]
            if Mp[index] != None and l - i <= Mp[index]:
                test2 = False
                break
            e = (e+1) % (len(HamiltonianCirculo) - 1)
            i += 1

    return test1 or test2, reversehc


def EsUnPosibleCmaino(path: list, snk: Snake, apl: Manzana, HamiltonianCirculo: list):

    RPOS = snk.Pos.copy()  # Las viejas posiciones de la serpiente antes del movimiento

    RDir = snk.Direccion  # La vieja dirección de la serpiente antes del movimiento

    for j in range(len(path)):
        snk.Change_Direccion(path[j])
        snk.Move()

    apl.Can_Eat(snk, False)

    test, reversehc = PuedeContinuarHamiltonianCirculo(snk, HamiltonianCirculo)

    snk.Pos = RPOS
    snk.Direccion = RDir
    snk.Length = len(RPOS)
    return test, reversehc


def MejorDir(snk: Snake, HamiltonianCirculo: list):
    head = snk.Pos[0]

    hi = HamiltonianCirculo.index(head)

    l = len(HamiltonianCirculo)

    BestDir = None

    less = False

    for dir in ALL_DIR:
        if PosibleDir(snk, dir):
            # nueva posición de la cabeza después de esa dirección
            Nhead = Mover(head, dir)
            if HamiltonianCirculo.index(Nhead) + (l if dir == snk.Direccion else 0) > hi:
                less = False
                BestDir = dir
                hi = HamiltonianCirculo.index(
                    Nhead) + (l if dir == snk.Direccion else 0)
            elif (BestDir == None or less == True) and dir == snk.Direccion:
                less = False
                BestDir = dir
                hi = HamiltonianCirculo.index(Nhead)
            elif BestDir == None:
                less = True
                BestDir = dir
                hi = HamiltonianCirculo.index(Nhead)

    return BestDir


def MapTour(snk: Snake, apl: Manzana, HamiltonianCirculo: list, HamiltonianPath: list, limit: int, ForbiddenPos: list):

    l = 0
    path = []
    TailPath = []

    RPos = snk.Pos.copy()
    RDir = snk.Direccion

    reversehc = False

    # representa si la serpiente comió una manzana mientras realiza la gira o no
    ComeManzana = False
    # la cantidad de pasos que tomó la serpiente después de comer una manzana
    PasosDespuesDeComer = 0

    while l < limit:
        dir = MejorDir(snk, HamiltonianCirculo)
        if snk.Pos[0] in ForbiddenPos:  # Se usa para eliminar bucles
            snk.Pos = RPos
            snk.Direccion = RDir
            snk.Length = len(RPos)
            return [], reversehc
        if dir != None:
            path.append(dir)
            snk.Change_Direccion(dir)
            snk.Move()
            TailPath.append(snk.ColaDir)
            if ComeManzana:
                PasosDespuesDeComer += 1
            if snk.Pos[0] == apl.Pos and not ComeManzana:
                snk.Append()
                ComeManzana = True
        else:
            break
        l += 1
    while len(path) > 0:
        if ComeManzana and PasosDespuesDeComer > 0:
            PasosDespuesDeComer -= 1
        elif ComeManzana:
            snk.Pos.pop()
            snk.Length = len(snk.Pos)
            ComeManzana = False
        c, reversehc = PuedeContinuarHamiltonianCirculo(
            snk, HamiltonianCirculo)
        if c:
            break
        dir = TailPath.pop()
        snk.Pos.reverse()
        snk.Change_Direccion(dir)
        snk.Move(True, True)
        path.pop()
        snk.Pos.reverse()
    snk.Pos = RPos
    snk.Direccion = RDir
    snk.Length = len(RPos)

    return path, reversehc


def ShowScore(scr, Color=TXT_CLR):
    global score

    # define la fuente y el tamaño
    font = pygame.font.SysFont("Verdana", 25, bold=True)
    # crea el texto con el puntaje
    score_text = font.render(f"Score: {score}", True, Color)
    score_rect = score_text.get_rect()  # obtiene el rectángulo del texto
    # posiciona el texto en la esquina superior izquierda
    score_rect.topleft = (10, 10)
    scr.blit(score_text, score_rect)  # dibuja el texto en la pantalla


Gana = False


def Inicio():
    global score
    score = 1

    pgm.init()

    global scr
    scr = pgm.display.set_mode(tamaño)  # la superficie del juego principal
    # Configura el título de la ventana
    pygame.display.set_caption("Snake")

    # Carga la imagen de fondo
    fondo = pygame.image.load("background.png")

    global Gana
    Gana = False  # representa si la serpiente ganó el juego o no

    C = pygame.time.Clock()

    global snk
    snk = Snake(Body_Padding=1)

    global apl
    apl = Manzana(scr, snk)

    # Especifica que la serpiente seguirá otro camino en lugar de seguir el ciclo hamiltoniano
    FollowPath = False

    path = []  # un camino que rompe el ciclo hamiltoniano y omite algunos pasos

    # Generando un mapa de la mitad de la longitud y el ancho
    Mp = Map.GenerateMap(GX // 2, GY // 2)

    G = Map.GenerateMapGraph(Mp)  # Convertir ese mapa en un gráfico

    G = Map.PrimsAlgorithm(G, 0)

    Maze = Map.GenerateHamiltonianMaze(G, Mp, GX, GY)

    HPos, HPath = Map.Maze2Cycle(Maze, GX, GY)

    global ReverseCycle
    ReverseCycle = False  # representa que revertiremos la dirección del ciclo hamiltoniano

    r = False

    # El índice de paso en la ruta del ciclo de Hamiltonia
    MoveIndex = HPos.index(snk.Pos[0])

    LoopDetector = []  # utilizado para detectar puntos de bucle
    ForbiddenPos = []  # uusado para romper loops

    SkippingPath = []  # El camino utilizado para omitir el ciclo hamiltoniano
    skip = True  # La serpiente solo seguirá al Hamiltoniancirco si Skip es falso

    while True:
        # llenando toda la pantalla para que la reiniciamos
        scr.fill(BCKG_Color)
        # Dibuja la imagen de fondo
        scr.blit(fondo, (0, 0))

        C.tick(70)  # corrige la velocidad de cuadro del juego

        try:
            if not FollowPath:  # trys para encontrar un camino hacia la manzana
                Mp = Map.GenerateEmptiesMap(
                    snk, apl, GX, GY, ExceptHead=True, Exceptapl=True)
                path = Map.AStarPathFinding(Mp, Mp.index(
                    snk.Pos[0]), Mp.index(apl.Pos), LEN//6)
        except:  # No existe tal camino
            path = None

        if path != None and not FollowPath and skip:
            # Prueba si la ruta es segura
            c, r = EsUnPosibleCmaino(path, snk, apl, HPos)
            if c:  # El camino es seguro
                FollowPath = True
                SkippingPath = []
                ReverseCycle = r
                LoopDetector = []
                ForbiddenPos = []
            else:  # El camino no es seguro
                r = None
        if not FollowPath:
            skipdir = None  # la dirección actual en la gira

            if len(SkippingPath) == 0 and skip:
                if snk.Pos[0] in LoopDetector:
                    ForbiddenPos.append(snk.Pos[0])
                # intenta encontrar un camino de gira alrededor del mapa
                SkippingPath, r = MapTour(
                    snk, apl, HPos, HPath, (GX+GY) * 2, ForbiddenPos)
                if len(SkippingPath) > 0:
                    LoopDetector.append(snk.Pos[0])
            if len(SkippingPath) > 0:
                if r != None:
                    ReverseCycle = r
                skipdir = SkippingPath.pop(0)

            if skipdir != None:
                snk.Change_Direccion(skipdir)
                snk.Move()
                if snk.Pos[0] in LoopDetector:
                    ForbiddenPos.append(snk.Pos[0])
            else:
                if ReverseCycle:
                    snk.Change_Direccion(HPath[MoveIndex-1])
                else:
                    snk.Change_Direccion(HPath[MoveIndex])
                snk.Move(ReverseCycle)
            # Ir al siguiente paso en el camino
            MoveIndex = HPos.index(snk.Pos[0])

        if FollowPath:  # la serpiente está siguiendo un camino hacia la manzana
            snk.Change_Direccion(path.pop(0))
            snk.Move()
        if FollowPath and len(path) == 0:  # Llegué al final del camino
            FollowPath = False
            MoveIndex = HPos.index(snk.Pos[0])

        snk.Draw(scr)  # Dibujar la serpiente en la pantalla

        # Agrega la serpiente si se comió la manzana y la reapareció
        apl.Can_Eat(snk)

        if len(snk) >= (LEN / 100) * 65 and skip:
            skip = False

        score = len(snk)
        ShowScore(scr)  # muestra la partitura

        if len(snk) == LEN:  # Ganó el juego
            Gana = True
            break
        else:
            apl.Draw()  # Dibujando la manzana en la pantalla

        if snk.ver_si_dead():  # ve si la serpiente está en una posición de muerte
            break

        pgm.display.flip()

        for ev in pygame.event.get():
            if ev.type == pgm.QUIT:
                pgm.quit()
                sys.exit()


def Game_Over():
    clock = pgm.time.Clock()

    font = pgm.font.SysFont("Verdana", 30, True)
    text = font.render("Perdiste . . .", False, TXT_CLR)

    TW = text.get_width()  # ancho de text

    global scr

    global snk
    snk.Draw(scr)

    scr.blit(text, ((w - TW) // 2, 0))

    ShowScore(scr)  # muestra la partitura

    pgm.display.flip()

    loop = True

    while loop:

        clock.tick(10)

        for ev in pgm.event.get():
            if ev.type == pgm.QUIT:
                pgm.quit()
                sys.exit()
            if ev.type == pgm.KEYDOWN and ev.key == pgm.K_r:
                pgm.quit()
                # Breaks para repetir el bucle de juegos (ver las últimas 3 líneas)
                loop = False
                break


def Game_Gana():
    clock = pgm.time.Clock()

    font = pgm.font.SysFont("Verdana", 30, True)
    text = font.render("¡GANASTE!", False, TXT_CLR)

    TW = text.get_width()  # ancho de texto
    TH = text.get_height()

    global scr

    global snk
    snk.Draw(scr)

    scr.blit(text, ((w - TW) // 2, (h - TH) // 2))  # draws el texto ganador

    ShowScore(scr)  # muestra la partitura

    pgm.display.flip()

    loop = True

    while loop:
        clock.tick(10)

        for ev in pgm.event.get():
            if ev.type == pgm.QUIT:
                pgm.quit()
                sys.exit()
            if ev.type == pgm.KEYDOWN and ev.key == pgm.K_r:
                pgm.quit()
                # se rompe para repetir el bucle de juegos (ver las últimas 3 líneas)
                loop = False
                break


while True:
    Inicio()  # Inicios el juego
    if not Gana:
        Game_Over()  # Si llegamos aquí, eso significa que ha sucedido un GameOver
    else:
        Game_Gana()  # Si llegamos aquí, eso significa que la IA ha ganado el juego
