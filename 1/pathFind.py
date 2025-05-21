# TODO : Create a code that reads in a graph and finds the shortest path between two nodes
import heapq

debug = True

Graph = {"N": 0, "M": 0, "edges": []}


def readData(fileName):
    # ! Filebol valo beolvasas
    
    with open(fileName, 'r') as file:
        firstLine = True
        for line in file:
            currentLine = line.strip().split()
            if firstLine:
                firstLine = False
                Graph["N"] = int(currentLine[0])
                Graph["M"] = int(currentLine[1])
            else:
                Graph["edges"].append({"x1": currentLine[0], "x2": currentLine[1], "w": int(currentLine[2])})

def printGraph():
    print("N:", Graph["N"])
    print("M:", Graph["M"])
    print("Edges:")
    for edge in Graph["edges"]:
        print(edge["x1"], edge["x2"], edge["w"])

def getNeighbours(node):
    neighbours = []
    for edge in Graph["edges"]:
        if edge["x1"] == node:
            neighbours.append((edge["x2"], edge["w"]))
        elif edge["x2"] == node:
            neighbours.append((edge["x1"], edge["w"]))
    return neighbours

def Dijkstra(startNode):
    distances = {i: float('inf') for i in range(1, Graph["N"] + 1)}
    distances[startNode] = 0

    priorityQueue = []
    heapq.heappush(priorityQueue, (0, startNode))
    visited = set()

    while priorityQueue:
        current_distance, current_node = heapq.heappop(priorityQueue)

        if current_node in visited:
            continue

        visited.add(current_node)

        for neighbour, weight in getNeighbours(current_node):
            if neighbour not in visited:
                new_distance = current_distance + weight
                if new_distance < distances[neighbour]:
                    distances[neighbour] = new_distance
                    heapq.heappush(priorityQueue, (new_distance, neighbour))

    return distances

readData("input.txt")
if debug:
    printGraph()

startNode = 1
shortest_paths = Dijkstra(startNode)
print("Legrövidebb utak a(z)", startNode, "csúcsból:", shortest_paths)
