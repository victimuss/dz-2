import random, time, matplotlib.pyplot as plt

class Graph:
    def __init__(self): #Инициализация
        self.adjacency_list = {} 

    def add_edge(self, u, v, weight): #Добавление вершины
        if u not in self.adjacency_list:
            self.adjacency_list[u] = []
        self.adjacency_list[u].append((v, weight))
        
        if v not in self.adjacency_list:
            self.adjacency_list[v] = []
        self.adjacency_list[v].append((u, weight))

    def len(self): #Вывод "длины" графа(Соня просила и ни разу не использовала :р)
        return(len(self.adjacency_list.keys()))
    
    def __str__(self):
        return "\n".join(f"{node}: {neighbors}" for node, neighbors in self.adjacency_list.items())


def read_graph(graph): 
    with open("hw5_data.txt", 'r') as file:
        for line in file: #Разбитие на линии
            parts = line.strip().split()
            if not parts:
                continue
            u = int(parts[0])
            for pair in parts[1:]:
                v, weight = map(int, pair.split(',')) #Добавление троек через метод add_edge
                graph.add_edge(u, v, weight)

# Пример использования
def dijkstra_slow(graph, start_vertex=1, total_vertices=200):
    # Инициализация расстояний
    distances = {v: float('inf') for v in range(0, total_vertices + 1)}
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

graph1 = Graph()
read_graph(graph1)
start_vertex = 1
shortest_paths = dijkstra_slow(graph1, start_vertex)

#Вывод результатов
for vertex in range(1, 201):
    print(f"Кратчайшее расстояние от {start_vertex} до {vertex}: {shortest_paths[vertex]}")  

      
def randomize(n, p, w):
    E = []
    for i in range(n):
        for j in range(i + 1, n):
            if random.random() < p:
                weight = random.randint(1, w + 1)     #добавление случайного веса
                E.append((i, j, weight))
    g = Graph()
    for i in E:
        g.add_edge(i[0],i[1],i[2])

    return g

g = randomize(6, 0.5, 19)
print(g)
print(g.len())
shortest_paths = dijkstra_slow(g, start_vertex)
for vertex in range(0, 5):
    print(f"Кратчайшее расстояние от {start_vertex} до {vertex}: {shortest_paths[vertex]}")       



class HeapItem:
    def __init__(self, v, p):
        self.v = v
        self.priority = p

    def __lt__(self, other):
        return self.priority < other.priority

    def __str__(self):
        return f"{self.v}/{self.priority}"


class Heap:
    def __init__(self):
        self.heap = []
        self.position = {}  # Трекинг позиций вершин

    def left_son(self, p):
        # возвращаем индекс левого сына элемента p
        return 2 * p + 1

    def right_son(self, p):
        # возвращаем индекс правого сына элемента p
        return 2 * p + 2

    def parent(self, p):
        # возвращаем индекс родителя элемента p
        return (p - 1) // 2 if p != 0 else 0

    def min_son(self, p):
        # возвращаем индекс минимального сына элемента p или -1, если p - лист
        left = self.left_son(p)
        right = self.right_son(p)
        n = len(self.heap)

        if left >= n:
            return -1
        if right >= n:
            return left

         # оба сына есть, возвращаем индекс с меньшим значением
        if self.heap[left] < self.heap[right]:
            return left
        else:
            return right

    def sift_up(self, p):
        # если мы в корне, то выходим
        if p == 0:
            return

        prnt = self.parent(p)
        # пока мы не в корне и текущий элемент меньше родительского, меняем их и поднимаемся выше
        while p > 0 and self.heap[p] < self.heap[prnt]:
            self.position[self.heap[p].v] = prnt
            self.position[self.heap[prnt].v] = p
            self.heap[p], self.heap[prnt] = self.heap[prnt], self.heap[p]
            p = prnt
            prnt = self.parent(p)

    def sift_down(self, p):
        # пока мы не в листе и текущий элемент больше минимального из сыновей,
        # меняем их местами и погружаемся ниже
        minCh = self.min_son(p)
        while minCh != -1 and self.heap[p] > self.heap[minCh]:
            self.position[self.heap[p].v] = minCh
            self.position[self.heap[minCh].v] = p
            self.heap[p], self.heap[minCh] = self.heap[minCh], self.heap[p]
            p = minCh
            minCh = self.min_son(p)

    def add(self, x):
        # добавляем элемент x в кучу
        if x.v in self.position:
            pos = self.position[x.v]
            if x.priority < self.heap[pos].priority:
                self.heap[pos].priority = x.priority
                self.sift_up(pos)
            return

        self.heap.append(x)
        self.position[x.v] = len(self.heap) - 1
        self.sift_up(len(self.heap) - 1)

    def get_min(self):
        # возвращаем значение минимального элемента, удаляя его из кучи
        if not self.heap:
            return None

        min_elem = self.heap[0]
        del self.position[min_elem.v]

        last_elem = self.heap.pop()
        if self.heap:
            self.heap[0] = last_elem
            self.position[last_elem.v] = 0
            self.sift_down(0)
        return min_elem

    def __str__(self):
        # печать массива с бинарным деревом кучи
        return ' '.join(map(str, self.heap))  # Убрана сортировка по номеру

def Dijkstra_fast(graph, start_vertex=1):
    # Инициализация расстояний и предшественников
    distances = {v: float('inf') for v in graph.adjacency_list}
    predecessors = {v: None for v in graph.adjacency_list}
    distances[start_vertex] = 0

    # Создаем кучу и добавляем все вершины
    heap = Heap()
    for v in graph.adjacency_list:
        heap.add(HeapItem(v, distances[v]))

    # Основной цикл алгоритма
    while heap.heap:
        # Извлекаем вершину с минимальным расстоянием
        u_item = heap.get_min()
        if u_item is None:
            break
            
        current_vertex = u_item.v
        current_dist = u_item.priority

        # Релаксация всех соседей
        for neighbor, weight in graph.adjacency_list.get(current_vertex, []):
            new_dist = current_dist + weight
            if new_dist < distances[neighbor]:
                # Обновляем расстояния и добавляем в кучу
                distances[neighbor] = new_dist
                predecessors[neighbor] = current_vertex
                heap.add(HeapItem(neighbor, new_dist))

    # Конвертируем в списки для совместимости
    sorted_vertices = sorted(graph.adjacency_list.keys())
    distance_list = [distances[v] for v in sorted_vertices]
    predecessor_list = [predecessors[v] for v in sorted_vertices]

    return distance_list, predecessor_list

def benchmark(
    generate_graph_func,  # Функция генерации графа: generate_graph(n) -> graph
    DijkstraSlow,           # Алгоритм 1: algorithm1(graph) -> result
    DijkstraFast,           # Алгоритм 2: algorithm2(graph) -> result
    A=10,                 # Минимальное количество вершин
    B=100,                # Максимальное количество вершин
    step=10,              # Шаг изменения n
    graphs_per_n=5,       # Количество графов для усреднения на каждое n
):
    results = {"n": [], "DijkstraSlow": [], "DijkstraFast": []}
    
    for n in range(A, B + 1, step):
        total_time_alg1 = 0.0
        total_time_alg2 = 0.0
        
        for _ in range(graphs_per_n):
            graph = generate_graph_func(n,p=random.random(),w=random.randint(A,B))  # Генерация графа
            
            # Замер времени для медленного Дейкстры
            start = time.perf_counter()
            DijkstraSlow(graph)
            total_time_alg1 += time.perf_counter() - start
            
            # Замер времени для быстрого Дейкстры
            start = time.perf_counter()
            DijkstraFast(graph)
            total_time_alg2 += time.perf_counter() - start
        
        # Усреднение результатов
        results["n"].append(n)
        results["DijkstraSlow"].append(total_time_alg1 / graphs_per_n)
        results["DijkstraFast"].append(total_time_alg2 / graphs_per_n)
    
    # Построение графиков
    plt.figure(figsize=(10, 6))
    plt.plot(results["n"], results["DijkstraSlow"], "o-", label="DijkstraSlow 1")
    plt.plot(results["n"], results["DijkstraFast"], "s-", label="DijkstraFast 2")
    plt.xlabel("Количество вершин (n)", fontsize=12)
    plt.ylabel("Среднее время (сек)", fontsize=12)
    plt.title("Зависимость времени выполнения от n", fontsize=14)
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.legend()
    plt.show()
    
    return results
benchmark(randomize, dijkstra_slow, Dijkstra_fast)





    

