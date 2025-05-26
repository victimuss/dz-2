class Graph:
    def __init__(self):
        self.g_list = {}

    def add_edge(self, u, v, weight):
        if u not in self.g_list:
            self.g_list[u] = []
        self.g_list[u].append((v, weight))
        if v not in self.g_list:
            self.g_list[v] = []
        self.g_list[v].append((u, weight))

    def __str__(self):
        return "\n".join(f"{node}: {neighbors}" for node, neighbors in self.g_list.items())
sonya = Graph()
def read_graph():
    with open('hw5_data.txt','r') as file:
        for line in file:
            parts = line.strip().split()
            u = parts[0]
            for pair in parts[1::]:
                v,weight =  map(int,pair.split(','))
                sonya.add_edge(u,v,weight)
                