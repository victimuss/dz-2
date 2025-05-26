class Graph:
    def __init__(self):
        self.adjacency_list = {} 

    def add_edge(self, u, v, weight):
        if u not in self.adjacency_list:
            self.adjacency_list[u] = []
        self.adjacency_list[u].append((v, weight))
        
        if v not in self.adjacency_list:
            self.adjacency_list[v] = []
        self.adjacency_list[v].append((u, weight))
    def __str__(self):
        return "\n".join(f"{node}: {neighbors}" for node, neighbors in self.adjacency_list.items())
def read_graph(graph):
    with open("hw5_data.txt", 'r') as file:
        for line in file:
            parts = line.strip().split()
            if not parts:
                continue
            u = int(parts[0])
            for pair in parts[1:]:
                v, weight = map(int, pair.split(','))
                graph.add_edge(u, v, weight)

# Пример использования
def dijkstra_slow(graph, start_vertex, total_vertices=200):
    # Инициализация расстояний
    distances = {v: float('inf') for v in range(1, total_vertices + 1)}
    distances[start_vertex] = 0
    visited = [False] * (total_vertices + 1)  # Индексы 0..200

    for _ in range(total_vertices):
        # Находим вершину с минимальным расстоянием
        min_dist = float('inf')
        current_vertex = None
        for v in range(1, total_vertices + 1):
            if not visited[v] and distances[v] < min_dist:
                min_dist = distances[v]
                current_vertex = v

        if current_vertex is None:
            break  # Все оставшиеся вершины недостижимы

        visited[current_vertex] = True

        # Обновляем расстояния для соседей
        if current_vertex in graph.adjacency_list:
            for (neighbor, weight) in graph.adjacency_list[current_vertex]:
                if distances[neighbor] > distances[current_vertex] + weight:
                    distances[neighbor] = distances[current_vertex] + weight

    # Заменяем inf на -1 для недостижимых вершин
    for v in distances:
        if distances[v] == float('inf'):
            distances[v] = -1

    return distances

# Чтение графа из файла (замени 'input.txt' на путь к файлу)
graph1 = Graph()
read_graph(graph1)
start_vertex = 1
shortest_paths = dijkstra_slow(graph1, start_vertex)

#Вывод результатов
for vertex in range(1, 201):
    print(f"Кратчайшее расстояние от {start_vertex} до {vertex}: {shortest_paths[vertex]}")        