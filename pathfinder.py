#Dimand Edvard
import math
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import time
import queue
import sys
from heapq import heappush, heappop

coord_x = []
coord_y = []
coord_z = []
values = []
grid = []
visited =[]
distances = []
parents = []
nr_of_points = 0
x_start = 0
y_start = 0
z_start = 0
x_end = 0
y_end = 0
z_end = 0
min_x = 0
max_x = 0
min_y = 0
max_y = 0

#check whether a neighbour can be traveled through
def isValid(x,y):
    global grid, visited, min_x, max_x, min_y, max_y
    #we check whether the node is outside of bounds/was already visited/is an obstacle
    if(x < min_x or x > max_x or y < min_y or y > max_y or isVisited(x,y) or grid[x][y] == None):
        return False
    return True

#mark a node as visited
def setVisited(i, j):
    global visited
    visited[i][j] = True
    return 

#check whether a node was visited
def isVisited(i,j):
    global visited
    return visited[i][j] == True

def get_neighbours(i,j):
    neighbours = []
    #upper row:
    if(isValid(i-1, j-1)):
        neighbours.append([i-1,j-1])
    if(isValid(i-1, j)):
        neighbours.append([i-1,j])
    if(isValid(i-1, j+1)):
        neighbours.append([i-1,j+1])
    #left and right neighbour:
    if(isValid(i, j-1)):
        neighbours.append([i,j-1])
    if(isValid(i, j+1)):
        neighbours.append([i,j+1])
    #lower row:
    if(isValid(i+1, j-1)):
        neighbours.append([i+1,j-1])
    if(isValid(i+1, j)):
        neighbours.append([i+1,j])
    if(isValid(i+1, j+1)):
        neighbours.append([i+1,j+1])
    #we return the valid neighbours:
    return neighbours

    
def get_path(start_x, start_y, end_x, end_y):
    global parents

    path = [[end_x,end_y]]
    x,y = parents[end_x][end_y]
    path.append([x,y])
    f = open("test.txt", "w")
    f.write("%i,%i\n" %(x,y))

    while(x!=start_x or y!=start_y):
        x, y = parents[x][y]
        path.append([x,y])
        f.write("%i,%i\n" %(x,y))

    f.close()
    return path



def init_env():
    global grid, distances, parents,visited,min_x, max_x, min_y, max_y
    #all obstacles are marked with 'X'
    max_dimension = 600
    grid = [[ None for i in range(max_dimension)] for i in range(max_dimension)]
    distances = [[ float('inf') for i in range(max_dimension)] for i in range(max_dimension)]
    parents = [[ None for i in range(max_dimension)] for i in range(max_dimension)]
    visited = [[ False for i in range(max_dimension)] for i in range(max_dimension)]
    min_x = int(min(coord_x))
    max_x = int(max(coord_x))
    min_y = int(min(coord_y))
    max_y = int(max(coord_y))
    for i in range(nr_of_points):
        #if the node is not an obstacle
        if(values[i] == 0):
            x = int(coord_x[i])
            y = int(coord_y[i])
            grid[x][y] = coord_z[i]

def read_surface():
    global coord_x, coord_y, coord_z, values, nr_of_points
    f = open("./surface.txt","r")
    contents = f.read().split("\n")
    for line in contents:
        x,y,z,val = [float(x) for x in line.split()]
        coord_x.append(x)
        coord_y.append(y)
        coord_z.append(z)
        values.append(val)
        nr_of_points += 1
    f.close()

def read_points():
    global x_start,y_start,z_start,x_end,y_end,z_end 
    f = open("./points.txt","r")
    contents = f.read().split()
    x_start,y_start,z_start,x_end,y_end,z_end = [float(x) for x in contents] 
    f.close()

def plot_roadmap3D():
    x = np.reshape(coord_x, (250, 250))
    y = np.reshape(coord_y, (250, 250))
    z = np.reshape(coord_z, (250, 250))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(x, y, z, alpha=0.5)
    ax.scatter(x_start, y_start, z_start, marker='x', color='green')
    ax.scatter(x_end, y_end, z_end, marker='x', color='red')
    ax.set_title("Utkereses")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


#DIJKSTRA WITH DISTANCE being considered as 1 step

def dijkstra():
    global x_start, y_start, x_end, y_end, visited, parents
    read_surface()
    read_points()
    init_env()

    x = np.reshape(coord_x, (250, 250))
    y = np.reshape(coord_y, (250, 250))
    z = np.reshape(coord_z, (250, 250))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, alpha=0.5)
    ax.scatter(x_start, y_start, z_start, marker='x', color='red',s=200)
    ax.scatter(x_end, y_end, z_end, marker='x', color='red',s=200)
    ax.set_title("Utkereses - A*")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    x_0,y_0 = int(x_start), int(y_start)
    x_1,y_1 = int(x_end), int(y_end)
    print("Searching for path between:[",x_0,",",y_0,"] and [",x_1,",",y_1,"]")
    print("Dijkstra's algorithm starting...")
    t0 = time.time()
    nodes = queue.Queue(80000)
    distances[x_0][y_0] = 1
    nodes.put([x_0,y_0])
    depth = 3000
    while(not nodes.empty()):
        x, y = nodes.get()
        if (x == x_1 and y == y_1):
            nodes.queue.clear()
            break
        visited[x][y] = True
        neighbours = get_neighbours(x,y)
        for neighbour in neighbours:
            setVisited(neighbour[0],neighbour[1])
            temp = distances[x][y] + 1 #TODO: change distance
            if temp < distances[neighbour[0]][neighbour[1]] :
                distances[neighbour[0]][neighbour[1]] = temp
                parents[neighbour[0]][neighbour[1]] = [x,y]
            nodes.put(neighbour)


    t1 = time.time()
    print("Dijkstra solved in:", t1-t0, " seconds")
    print("Distance:")
    print(distances[x_1][y_1])
    print("Searching path between",x_0, y_0,"=>", x_1, y_1)
    path = get_path(x_0, y_0, x_1, y_1)
    
    for x_path,y_path in path:
        ax.scatter(x_path,y_path,grid[x_path][y_path],marker='|',color='green')
    plt.show()

#A* where path is chosen based on euclidean distance

def a_star_euclidean():
    global x_start, y_start, x_end, y_end, visited, parents
    read_surface()
    read_points()
    init_env()

    x = np.reshape(coord_x, (250, 250))
    y = np.reshape(coord_y, (250, 250))
    z = np.reshape(coord_z, (250, 250))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, alpha=0.5)
    ax.scatter(x_start, y_start, z_start, marker='x', color='red',s=200)
    ax.scatter(x_end, y_end, z_end, marker='x', color='red',s=200)
    ax.set_title("Utkereses - A* - Euclidean")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    x_0,y_0 = int(x_start), int(y_start)
    x_1,y_1 = int(x_end), int(y_end)

    print("Searching for path between:[",x_0,",",y_0,"] and [",x_1,",",y_1,"]")
    print("A* algorithm starting...")
    t0 = time.time()
    # nodes = queue.Queue(80000)
    distances[x_0][y_0] = 0
    # 
    # nodes.put([x_0,y_0])
    l = []
    heappush(l,(0.0,[x_0,y_0]))
    
    depth = 3000
    while(l):
        temp, coord = heappop(l)
        x, y = coord
        if (x == x_1 and y == y_1):
            break
        visited[x][y] = True
        neighbours = get_neighbours(x,y)
        for neighbour in neighbours:
            n_x, n_y = neighbour
            # setVisited(n_x, n_y)
            # euclidean distance instead of just +1 step
            temp = distances[x][y] + get_distance(x, y, n_x, n_y)
            if temp < distances[n_x][n_y] :
                distances[n_x][n_y] = temp
                parents[n_x][n_y] = [x,y]
                temp = temp + get_distance(n_x, n_y, x_1, y_1)
                heappush(l, (temp, neighbour))


    t1 = time.time()
    print("A* solved in:", t1-t0, " seconds")
    print("Distance:")
    print(distances[x_1][y_1])
    print("Searching path between",x_0, y_0,"=>", x_1, y_1)
    path = get_path(x_0, y_0, x_1, y_1)
    
    for x_path,y_path in path:
        ax.scatter(x_path,y_path,grid[x_path][y_path],marker='|',color='green')
    plt.show()



def get_distance(x_a,y_a, x_b,y_b):
    global grid, x_end, y_end

    x_a = int(x_a)
    y_a = int(y_a)

    x_b = int(x_b)
    y_b = int(y_b)

    z_a = grid[x_a][y_a]
    z_b = grid[x_b][y_b]
    return math.sqrt((x_b - x_a)**2 + (y_b - y_a)**2 + (z_b - z_a)**2)

def get_height_diff(x_a,y_a, x_b,y_b):
    global grid, x_end, y_end
    
    z_a = grid[x_a][y_a]
    z_b = grid[x_b][y_b]
    if(z_b-z_a) < 0:
        return 0
    return z_b - z_a

def a_star_energy():
    global x_start, y_start, x_end, y_end, visited, parents
    read_surface()
    read_points()
    init_env()

    x = np.reshape(coord_x, (250, 250))
    y = np.reshape(coord_y, (250, 250))
    z = np.reshape(coord_z, (250, 250))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, alpha=0.5)
    ax.scatter(x_start, y_start, z_start, marker='x', color='red',s=200)
    ax.scatter(x_end, y_end, z_end, marker='x', color='red',s=200)
    ax.set_title("Utkereses - A* - Energy")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    x_0,y_0 = int(x_start), int(y_start)
    x_1,y_1 = int(x_end), int(y_end)

    print("Searching for path between:[",x_0,",",y_0,"] and [",x_1,",",y_1,"]")
    print("A* algorithm starting...")
    t0 = time.time()
    distances[x_0][y_0] = 0
    l = []
    heappush(l,(0.0,[x_0,y_0]))
    
    depth = 3000
    while(l):
        temp, coord = heappop(l)
        x, y = coord
        if (x == x_1 and y == y_1):
            break
        visited[x][y] = True
        neighbours = get_neighbours(x,y)
        for neighbour in neighbours:
            n_x, n_y = neighbour
            # setVisited(n_x, n_y)
            # euclidean distance instead of just +1 step
            temp = distances[x][y] + get_height_diff(x, y, n_x, n_y)
            if temp < distances[n_x][n_y] :
                distances[n_x][n_y] = temp
                parents[n_x][n_y] = [x,y]
                temp = temp + get_height_diff(n_x, n_y, x_1, y_1)
                heappush(l, (temp, neighbour))


    t1 = time.time()
    print("A* solved in:", t1-t0, " seconds")
    print("Energy needed:")
    print(distances[x_1][y_1])
    print("Searching path between",x_0, y_0,"=>", x_1, y_1)
    path = get_path(x_0, y_0, x_1, y_1)
    
    for x_path,y_path in path:
        ax.scatter(x_path,y_path,grid[x_path][y_path],marker='|',color='green')
    plt.show()

def main():
    algorithm_nr = int(sys.argv[1])
    if(algorithm_nr == 1):
        dijkstra()
        return
    if(algorithm_nr == 2):
        a_star_euclidean()
        return
    if(algorithm_nr == 3):
        a_star_energy()
        return
    
main() 