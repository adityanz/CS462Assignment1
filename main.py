from graph import Graph
def main():
    graph = Graph()
    graph.readFromFile("soc-tribes.edges")
    print("BFS")
    graph.breadthFirstSearch(1, 10)
    print("DFS")
    graph.depthFirstSearch(1, 10)
    print("Djikstra")
    graph.djikstra(1)
    print("Prim")
    graph.prim(1)
main()