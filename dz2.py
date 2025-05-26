import random
import time

import random
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

# Вывод результатов
# for vertex in range(1, 201):
#     print(f"Кратчайшее расстояние от {start_vertex} до {vertex}: {shortest_paths[vertex]}")        
def randomize(n, p, w):
    V = set(range(1,n))
    E = []
    for i in range(n):
        for j in range(i + 1, n):
            if random.random() < p:
                weight = random.randint(1, w + 1)     #добавление случайного веса
                weight = random.randint(1, w + 1)        #добавление случайного веса
                E.append((i, j, weight))
    g = Graph()
    for i in E:
        g.add_edge(i[0],i[1],i[2])

    return g

g = randomize(6, 4, 19)
print(g)

shortest_paths = dijkstra_slow(g, start_vertex)
for vertex in range(0, 6):
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
        return 2 * p + 1

    def right_son(self, p):
        return 2 * p + 2

    def parent(self, p):
        return (p - 1) // 2 if p != 0 else 0

    def min_son(self, p):
        left = self.left_son(p)
        right = self.right_son(p)
        n = len(self.heap)

        if left >= n:
            return -1
        if right >= n:
            return left

        if self.heap[left] < self.heap[right]:
            return left
        else:
            return right

    def sift_up(self, p):
        if p == 0:
            return

        prnt = self.parent(p)
        while p > 0 and self.heap[p] < self.heap[prnt]:
            self.position[self.heap[p].v] = prnt
            self.position[self.heap[prnt].v] = p
            self.heap[p], self.heap[prnt] = self.heap[prnt], self.heap[p]
            p = prnt
            prnt = self.parent(p)

    def sift_down(self, p):
        minCh = self.min_son(p)
        while minCh != -1 and self.heap[p] > self.heap[minCh]:
            self.position[self.heap[p].v] = minCh
            self.position[self.heap[minCh].v] = p
            self.heap[p], self.heap[minCh] = self.heap[minCh], self.heap[p]
            p = minCh
            minCh = self.min_son(p)

    def add(self, x):
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
        return ' '.join(map(str, self.heap))  # Убрана сортировка по номеру

def counting_sort_neighbors(neighbors):
    if not neighbors:
        return []
    max_v = max(n[0] for n in neighbors)
    count = [0] * (max_v + 1)
    for n in neighbors:
        count[n[0]] += 1
    for i in range(1, max_v + 1):
        count[i] += count[i-1]
    output = [0] * len(neighbors)
    for n in reversed(neighbors):
        output[count[n[0]]-1] = n
        count[n[0]] -= 1
    return output


def  Dijkstra_fast(G, s):
    # инициализируем массивы d и pi
    d, pi = Dijkstra_init(G, s)
    N = len(G)
    INF = N * 100

    # создаем кучу, кладем в нее все вершины и делаем приоритет 0 для вершины s
    heap = Heap()
    for v in range(N):
        heap.add(HeapItem(v, 0 if v == s else INF))

    # запускаем цикл, выполняющийся N раз, где N - количество вершин в графе
    for _ in range(N):
        # печатаем кучу
        print(heap)

        # находим вершину с минимальной жадной оценкой Дейкстры (с помощью кучи)
        u_item = heap.get_min()
        if u_item is None:
            break
        u = u_item.v

        neighbors = counting_sort_neighbors(G[u])
        # пробегаемся по всем ее соседям v и делаем релаксацию ребер (u,v)
        # не забываем менять жадную оценку не только в d, но и в куче!
        for v, cost in neighbors:
            if d[v] > d[u] + cost:
                d[v] = d[u] + cost
                pi[v] = u
                heap.add(HeapItem(v, d[v]))

    # возвращаем ответ
    return (d, pi)


    

