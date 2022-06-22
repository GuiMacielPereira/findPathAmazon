from turtle import st
import numpy as np
import matplotlib.pyplot as plt


def main():

    startP = [0, 0]
    endP = [9, 9]
    obstacles = [[9, 7], [8, 7], [6, 7], [6, 8], [7, 7], [7, 8], [3, 3]]

    # Option to include random points
    randomObstacles = False
    if randomObstacles:
        noRandPoints = 10
        obstacles = np.random.randint(0, 10, (noRandPoints, 2)).tolist()
        if startP in obstacles:
            obstacles.remove(startP)
        if endP in obstacles:
            obstacles.remove(endP)


    # Offset grid to make boundaries by placing obstacles
    N = np.max([endP[0] - startP[0], endP[1] - startP[1]]) + 1
    grid = np.zeros((N+4, N+4))
    grid[1:-2, [1, -2]] = 1     # Create boundaries by placing obstacles
    grid[[1, -2], 1:-2] = 1
    
    for (x, y) in obstacles:
        grid[x+2, y+2] = 1
    startP = list(np.array(startP) + 2)
    endP = list(np.array(endP) + 2)

    # Run main path finding algorithm
    pathPoints = findPath(grid, startP, endP)

    print(pathPoints)
    pathPoints = np.array(pathPoints)
    obstacles = np.argwhere(grid==1)
    plt.scatter(obstacles[:, 0], obstacles[:, 1], label="Obstacles")
    plt.scatter(pathPoints[:, 0], pathPoints[:, 1], label="Path")
    plt.legend()
    plt.show()
    return


def findPath(grid, startP, endP):

    pathPoints = []
    currP = startP
    pathPoints.append(currP)

    while currP != endP:

        nxtP = calcNextPoint(currP, endP)
        nxtX, nxtY = nxtP

        if grid[nxtX, nxtY] == 1:
            freePoints = []
            checkObs(grid, nxtP, [], freePoints)
            freePoints = removeDuplicates(freePoints)
            #removeIsolatedPoints(freePoints)
            
      
            enterP = currP if currP in freePoints else getEnterPoint(currP, freePoints)

            exitP = calcExitP(enterP, freePoints, endP)

            getPathPoint(currP, exitP, freePoints, pathPoints)

            currP = exitP

        else:
            pathPoints.append(nxtP)
            currP = nxtP
    
    return pathPoints

def getEnterPoint(currP, freePoints):
    for fp in freePoints:
        d = dist(currP, fp)
        if (d==1) or (d==np.sqrt(2)):
            return fp
    raise ValueError ("Enter point not found!")


def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def checkObs(grid, pCheck, pIgnore, freePoints):
    x, y = pCheck
    obsPoints = []

    for (nx, ny) in [(x, y+1), (x, y-1), (x+1, y), (x-1, y)]:
        if [nx, ny] in pIgnore:
            continue

        if grid[nx, ny] == 0:
            freePoints.append([nx, ny])
        elif grid[nx, ny] == 1:
            obsPoints.append([nx, ny])

        else:
            raise ValueError("Grid point must be either 0 or 1.")
    
    pIgnore.append(pCheck)

    for obsP in obsPoints:
        checkObs(grid, obsP, pIgnore, freePoints)
    return


def getPathPoint(currP, pEnd, freePoints, pathPoints):
    pathPoints.append(currP)

    if currP == pEnd:
        return

    for fp in freePoints:
        d = dist(currP, fp)

        if ((d==1) or (d==np.sqrt(2))) and (fp not in pathPoints):
            pathPoints.append(fp)
            nxtP = fp
            break
    
    getPathPoint(nxtP, pEnd, freePoints, pathPoints)


def removeDuplicates(freePoints):
    res = []
    for fp in freePoints:
        if fp not in res:
            res.append(fp)
    return res


def removeIsolatedPoints(freePoints):
    while True:
        isolatedPoints = []
        for fp in freePoints:
            if IsIsolated(fp, freePoints):
                isolatedPoints.append(fp)
        
        if len(isolatedPoints)==0:
            break

        for ip in isolatedPoints:
            freePoints.remove(ip)
    return


def IsIsolated(currP, freePoints):
    neighborsCount = 0
    for fp in freePoints:
        d = dist(fp, currP)
        if ((d==1) or (d==np.sqrt(2))):
            neighborsCount += 1

    if neighborsCount < 2:
        return True
    return False


def calcExitP(enterP, freePoints, endP):

    closest = 1000   # Big value to be replaced at first iteration
    desired = dist(enterP, endP)
    
    for fp in freePoints:

        if fp == enterP:
            continue

        curr = dist(fp, endP)

        if curr < closest:
            closest = curr
            bestP = fp

    return bestP


def calcNextPoint(currP, endP):

    res = [0, 0]
    for i, (currCoor, endCoor) in enumerate(zip(currP, endP)):
        if currCoor == endCoor:
            res[i] = currCoor
        elif currCoor > endCoor:
            res[i] = currCoor - 1
        elif currCoor < endCoor:
            res[i] = currCoor + 1
        else:
            raise ValueError("Error in determining next point")

    if res != endP:
        assert res != currP, f"res={res}, currP={currP}"
        assert res != [0, 0], f"Assignemnt did not work!"
    return res


if __name__ == "__main__":
    main()