import math
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils, math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
from itertools import permutations, combinations
def where_it_is(line, point):
    # verilen 3 noktayı saat yöne baz alınarak 
    # ortadaki noktanın sağında mı solunda mı kalındığına bakılır
    # sağında ise içbükey, solunda ise dışbükey olmuş olur.
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
    print(" icBukeyMi fonksiyonu çalıştı ")
    triPoint = []
    icBukeyUckenler = []
    points.extend(points[:2]) # A,B,C >> A,B,C,A,B
    ncoord = [] # içte kalmayan noktalar listesi
    for i in points:
        triPoint.append(i)
        tp = triPoint
        if len(triPoint) == 3: # koordinatlar 3'erli olarak sırayla alınır
            nerde = where_it_is((tp[0], tp[2]), tp[1])
            print(tp[1], ":::" , nerde, ":::" ,(tp[0], tp[2]))
            triPoint = tp[1:]
            if nerde == "right":
                icBukeyUckenler.append(tp) # tp[1] içte kalan noktadır!!
            elif nerde == "left":
                # ncoord.extend(tp[1:])
                ncoord.extend([tp[1]])
                
            else:
                # ncoord.extend(tp[1:])
                ncoord.extend([tp[1]])
                
    return ncoord, icBukeyUckenler




def order_points_old(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

# coord = [[0,2.22], [3.18, 4.04], [4.44, 2],[1.5, 3.5], [2, 0],[0, 0.56]]
# #coord = [[2,2],[2,4],[4,4], [4,2]] 

# coord = [[0,2.22], [3.18, 4.04], [4.44, 2],[2, 2], [2, 0],[0, 0.56], [1,1]]
# #npc = np.array(coord)7
# #result = order_points_old(npc)
# #print(result)
# coord = [
#         [653040, 4202600],
#         [653409, 4202600],
#         [653361, 4202385],
#         [653199, 4202142],
#         [652887, 4202238],
#         [652895, 4202365]
#         ]
# coord = [
#         [653200, 4202092],
#         [653175, 4201846],
#         [652980, 4201813],
#         [652945, 4201978],
#         ]
coord = [
        [653298, 4201883],
        [653459, 4201840],
        [653460, 4201440],
        [653500, 4201440],
        [653500, 4201245],
        [653340, 4201260],
        [653225, 4201750]
        ]
coord = [[c[1], c[0]] for c in coord] # x, y = y, x
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
def noktaIcindeMi(koodinatlar, bkNokta):
    print("-4-4-4-4-4-4-4-4-4-4-4-4")
    coords = koodinatlar
    cntr = bkNokta
    print(cntr) # bakılacak nokta
    totalAci = 0
    aciList = []
    ikiliList = []
    print("ikişerli liste")
    try:
        for i in range(len(coords)):
            ikili = coords[i], coords[i+1]
            ikiliList.append(ikili)
            print(ikili)
    except:
        pass
    sonIkili = coords[-1], coords[0]
    ikiliList.append(sonIkili)
    print(sonIkili)
    acilar = []
    print("--------------------------------------")
    for ikili in ikiliList:
        print(ikili, "############################")
        hip1 = uo(ikili[0], cntr)[0]
        print('hip1', hip1)
        hip2 = uo(ikili[1], cntr)[0]
        print('hip2', hip2)
        ui = uo(*ikili)[0] # karsi iki nokta arasi uzunluk
        h = distance_numpy(*np.array([ikili[0],ikili[1], cntr]))
        hg1 = h/hip1
        print("hg1: ", hg1)
        hg2 = h/hip2
        print("hg2: ", hg2)
        # h = hHesapla(ikili[0],ikili[1], cntr)
        # print("h me: ", h)
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
            print("ui: ", ui) # karsi iki nokta arasi uzunluk
            u1 = h/math.tan(math.radians(aci1))
            u2 = h/math.tan(math.radians(aci2))
            print("u1: ", u1)
            print("u2: ", u2)
            print("u1 + u2:", round((u1 + u2),3))
            if (round((u1 + u2),3)) == round(ui, 3):
                fAci = 180 - (aci1 + aci2)
            # if not (round((u1 + u2),3)) > round(ui, 3):
            #     fAci = 180 - (aci1 + aci2)
            else:
                print("------- hataliiii")
            if uo(cntr, ikili[0])[0] > uo(cntr, ikili[1])[0] and not (round((u1 + u2),3)) == round(ui, 3):
                print("teeee")
                h = distance_numpy(*np.array([cntr,ikili[0], ikili[1]]))
                fAci = math.degrees(math.asin(h/hip2))
            elif not (round((u1 + u2),3)) == round(ui, 3):
                h = distance_numpy(*np.array([cntr,ikili[1], ikili[0]]))
                fAci = math.degrees(math.asin(h/hip1))


        print("----->>> ", round(fAci,3))
        totalAci += fAci
        aciList.append(fAci)
        acilar.append(aci1)
        acilar.append(aci2)

    return aciList

        # print(aciList)
        # print(sum(aciList))
        # print(totalAci)
    
def enUzunKenar(coords, cntr):
    cn = coords
    icBukeyUckenList = []
    syc = 0
    while True: # iç bükey nokta kalmayana kadar devam et.
        print("syc : ", syc)
        coords, icBukeyUckenler = icBukeyMi(coords)
        icBukeyUckenList.extend(icBukeyUckenler)
        if icBukeyUckenler != []:
            print(icBukeyUckenler)
            acilar = noktaIcindeMi(*icBukeyUckenler, cntr)
            print(acilar)
            print(sum(acilar))
            if round(sum(acilar),3) == 360.000:
                print("koordinat alanın içerisinde kalmıyor!!!!!!!!!!!")
                return
        else:
            break
        syc +=1
    # print("coords_N: ", cn)
    # print("coords: ", coords)
    # print("icBukeyUckenler: ", icBukeyUckenList)
    if list(cntr) in coords: # eğer bakılacak nokta koordinatların içersindeyse brak
        print(360)
        return

    acilar = noktaIcindeMi(coords, cntr)
    print(acilar)
    print(sum(acilar))
    


def calcDegree(a,b):
    print("___>>>> açı hesaplanıyor <<<<___")
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



#enUzunKenar(coord)
#coord = [[2,4], [4, 4], [4,2],[2,2]]   
pts = []
print("koordinatlar: \n")
print(coord)
origin = centerOfPoints(coord)
print("koordinatların merkezi: \n", origin)
refvec = [0,1]
print("koordinatların saat yönünde sıralanmış hali: \n")
# result = sorted(coord, key=cwd)
result = coord
print(result)
enUzunKenar(result[::-1], (4201264,653455))
#363.97

points = [(3,4), (4,7),(7,8),(8,4),(12,3),(10,1),(5,2)]
print(solve(coord))


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)