

from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils, math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
from itertools import permutations, combinations
def where_it_is(line, point):
    a = line[0]
    b = line[1]
    c = point
    ax,ay = a
    bx,by = b
    cx,cy = c
    val = ((bx - ax)*(cy - ay) - (by - ay)*(cx - ax))
    if val > 0:
        return "left"
    elif val < 0:
        return "right"
    else:
        return "point is on the line"
def icBukeyMi(points):
    triPoint = []
    icBukeyUckenler = []
    points.extend(points[:2])
    print("points: ", points)
    syc = len(points)-2
    sp = 0
    for i in range(len(points)-2):
        tp = points[i:i+3]
        nerde = where_it_is((tp[0], tp[2]), tp[1])
        print(tp[1], ":::" , nerde, "---->>", tp)
        syc -= 1
        if nerde == "right":
            icBukeyUckenler.append(tp)
    print("kere: ", sp)
    print(len(points)-2)
    return icBukeyUckenler




def order_points_old(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

coord = [[0,2.22], [3.18, 4.04], [4.44, 2],[1.5, 3.5], [2, 0],[0, 0.56]]
#coord = [[2,2],[2,4],[4,4], [4,2]] 

coord = [[0,2.22], [3.18, 4.04], [4.44, 2],[2, 2], [2, 0],[0, 0.56], [1,1]]
coord = [[0,2.22], [3.18, 4.04], [4.44, 2],[2, 2], [2, 0],[0, 0.56], [1,1]]

#npc = np.array(coord)7
#result = order_points_old(npc)
#print(result)
# coord = [
#         [653040, 4202600],
#         [653409, 4202600],
#         [653361, 4202385],
#         [653199, 4202142],
#         [652887, 4202238],
#         [652895, 4202365]
#         ]
coord = [
        [653200, 4202092],
        [653175, 4201846],
        [652980, 4201813],
        [652945, 4201978],
        ]

        # [653460, 4201440],
# coord = [
#         [653298, 4201883],
#         [653459, 4201840],
#         [653460, 4201440],
#         [653500, 4201440],
#         [653500, 4201245],
#         [653340, 4201260],
#         [653225, 4201750]
#         ]

coord = [
    [632540, 4199217],
    [632633, 4199362],
    [632813, 4199535],
    [633104, 4200000],
    [633020, 4199565],
    [632999, 4199793],
    [633250, 4200000],
    [633313, 4199749],
    [633168, 4199438],
    [633075, 4199375],
    [633042, 4199314],
    [632796, 4199071],
    [632750, 4199095],
    [632632, 4199080]
]

coord = [[c[1], c[0]] for c in coord]
def centerOfPoints(coord):
    xList = []
    yList = []
    for c in coord:
        xList.append(c[0])
        yList.append(c[1])
    center = sum(xList)/len(xList), sum(yList)/len(yList)
    return center

#cntr = centerOfPoints(coord)
#print(cntr)


#perm = permutations(coord,2)
#for p in perm:
#  print(p)
def uo(nokta1, nokta2):
    x1, y1 = nokta1
    x2, y2 = nokta2
    xd = abs(x2-x1)
    yd = abs(y2-y1)
    u = ((xd**2)+(yd**2))**0.5
    u = math.hypot(x2-x1, y2-y1)
    ortaNokta = (x1+x2)/2, (y1+y2)/2
    return u, ortaNokta

def u(a,b):
    return np.linalg.norm(a-b)
def enUzunKenar(coords, cntr):
    cn = coords
    
    
    print("coords_N: ", cn)
    cnn = [] 

    
    for c in coords:
        if c not in cnn:
            cnn.append(c)
    cnn.append(cnn[0])
    icBukeyUckenler = icBukeyMi(coords)
    print("coords: ", cnn)
    
    if len(icBukeyUckenler) !=0:
        k = [icb[1] for icb in icBukeyUckenler]
        coords = [c for c in cn if c not in k]
    else:
        coords = cnn
    print("final coords: ", coords)
    if list(cntr) in coords:
        print(360)
        return

    print(cntr)
    totalAci = 0
    aciList = []
    ikiliList = []
    try:
        for i in range(len(coords)):
            ikili = coords[i], coords[i+1]
            ikiliList.append(ikili)
    except:
        pass
    # sonIkili = coords[-1], coords[0]
    # ikiliList.append(sonIkili)
    acilar = []
    for ikili in ikiliList:
        print(ikili)
        hip1 = uo(ikili[0], cntr)[0]
        print('hip1', hip1)
        hip2 = uo(ikili[1], cntr)[0]
        print('hip2', hip2)
        ui = uo(*ikili)[0] # karsi iki nokta arasi uzunluk
        h = distance_numpy(*np.array([ikili[0],ikili[1], cntr]))
        h = pdt(cntr, ikili)
        print("h de: ", h)
        # h = distance_numpy(*np.array([ikili[0],ikili[1], cntr]))
        if (h/hip1 > 1) or (h/hip1 < -1):
            print("------ if ------")
            aci2 = calcDegree(h, hip2)
            print('aci2', aci2)
            u2 = h/math.tan(aci2)
            print("u2: ", u2)
            u1 = ui - u2
            print("u1: ", u1)
            if (u1 + u2)> ui:
                print("-------- hatalı açı")
            aci2 = math.atan2(h, u1)
            print('aci1', aci1)
            fAci = 180 - (aci1 + aci2)

        elif (h/hip2 > 1) and (h/hip2 < -1):
            print("------ elif -----")
            aci1 = calcDegree(h, hip1)
            print('aci1', aci1)
            u1 = h/math.tan(aci1)
            if u1 > ui:
                print("stop")
            u2 = ui - u1
            aci2 = math.atan2(h, u2)
            print('aci2', aci2)
            fAci = 180 - (aci1 + aci2)
        else:
            print("----else")
            aci1 = calcDegree(h, hip1)
            print("aci1: ", aci1)
            aci2 = calcDegree(h, hip2)
            print("aci2: ", aci2)
            print("ui: ", ui)
            u1 = h/math.tan(math.radians(aci1))
            u2 = h/math.tan(math.radians(aci2))
            print("u1: ", u1)
            print("u2: ", u2)
            print("u1 + u2:", round((u1 + u2),3))
            if not (round((u1 + u2),3)) > round(ui, 3):
                fAci = 180 - (aci1 + aci2)
            else:
                print("------- hataliiii")
                if uo(cntr, ikili[0])[0] > uo(cntr, ikili[1])[0]:
                    print("teeee")
                    h = distance_numpy(*np.array([cntr,ikili[0], ikili[1]]))
                    # h = pdt(cntr, ikili)
                    fAci = math.degrees(math.asin(h/hip2))
                else:
                    h = distance_numpy(*np.array([cntr,ikili[1], ikili[0]]))
                    fAci = math.degrees(math.asin(h/hip1))


        print("----->>> ", round(fAci,3))
        totalAci += fAci
        aciList.append(fAci)
        acilar.append(aci1)
        acilar.append(aci2)



    if len(coords) > 5:
        k = "0."+"0"*(len(cn)-4)+"1"
        k = float(k)
    else:
        k = 0
    print(totalAci-k)
    
    
    for ikili in zip(ikiliList, aciList):
        print(ikili)
    print(len(ikiliList))
    print("icBukeyUckenler: ")
    for icb in icBukeyUckenler:
        print(icb)


def calcDegree(a,b):
    print("h: ", a)
    print("hip: ", b)
    print(f"arcsin({a/b})")

    try:
        return math.asin(a/b)*(180/math.pi)
        return math.degrees(np.arcsin(*np.array([a/b])))
    except:
        return math.degrees(math.asin(a/b))
# from: https://gist.github.com/nim65s/5e9902cd67f094ce65b0
def distance_numpy(A, B, P):
    """ segment line AB, point P, where each one is an array([x, y]) """
    if all(A == P) or all(B == P):
        return 0
    if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > math.pi / 2:
        return norm(P - A)
    if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > math.pi / 2:
        return norm(P - B)
    return norm(cross(A-B, A-P))/norm(B-A)
def pdt(pt, line):
    p1=np.array(line[0])
    p2=np.array(line[1])
    p3=np.array(pt)
    d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
    return abs(d)


#enUzunKenar(coord)
#coord = [[2,4], [4, 4], [4,2],[2,2]]   
import math
pts = []
origin = centerOfPoints(coord)
refvec = [0,1]
def cwd(point):
    vektor = [point[0] - origin[0], point[1] - origin[1]]
    lenvector = math.hypot(vektor[0], vektor[1])
    if lenvector == 0:
        return math.pi, 0
    normalized = [vektor[0]/lenvector, vektor[1]/lenvector]
    dotprod = normalized[0]*refvec[0] + normalized[1]*refvec[1]
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]
    angle = math.atan2(diffprod, dotprod)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    return angle, lenvector

def hHesapla(nokta1, nokta2, cntr):
    aX, aY = nokta1
    bX, bY = nokta2
    try:
        #m = (aY-bY)/(aX-bX)
        m = (bY-aY)/(bX-aX)
    except:
        m=1
    x, y = cntr
    a, b = 1, -m
    # y - b =m(x- a)
    pay = abs(y - b - m*x + m*a) # |y - b - m.x + m.a|
    payda = ((a**2)+(b**2))**0.5
    h = pay/payda
    return h



def get_angle(a, b, c):
    angle = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    return angle + 360 if angle < 0 else angle
def solve(points):
    n = len(points)
    for i in range(len(points)):
        p1 = points[i-2]
        p2 = points[i-1]
        p3 = points[i]
    if get_angle(p1, p2, p3) > 180:
        return True
    return False

points = [(3,4), (4,7),(7,8),(8,4),(12,3),(10,1),(5,2)]
print(solve(coord))


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)

result = sorted(coord, key=cwd)
print(result)
print("saat yonunde: ", result)
cntr = centerOfPoints(result)
print(cntr)
enUzunKenar(result, cntr)
#363.97
print(cntr)
