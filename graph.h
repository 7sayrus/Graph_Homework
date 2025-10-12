//
// Created by sesab on 16.03.2024.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <set>

class graph {
private:
    int n; // Переменная для хранения количества вершин графа
    std::vector<std::vector<int>> adj; // Вектор векторов для хранения смежности графа
    std::vector<std::pair<int, int>> edge; // Вектор пар для хранения рёбер графа (если граф невзвешенный)
    std::vector<std::tuple<int,int,int>> edges; // Вектор кортежей (первая вершины, вторая, длина пути)
public:

    graph(int n) { // Конструктор класса с одним параметром (количеством вершин)
        this->n = n; // Присваивание переданного количества вершин переменной n
        adj.resize(n); // Изменение размера вектора смежности для хранения n вершин
    }

    void add_edge(int first, int second) { // Метод для добавления ребра между вершинами
        adj[first].push_back(second); // Добавление второй вершины в список смежности первой
        adj[second].push_back(first); // Добавление первой вершины в список смежности второй
        edge.push_back({first, second}); // Добавление ребра в вектор рёбер
    }


    void add_edge(int first, int second, int weight) { // Метод для добавления ребра между вершинами + вес ребра
        // Проверяем, есть ли уже такое ребро
        bool edge_exists = false;
        for (auto &e: edges) {
            // Проверяем, существует ли ребро между вершинами first и second (или между second и first)
            if ((std::get<0>(e) == first && std::get<1>(e) == second) ||
                (std::get<0>(e) == second && std::get<1>(e) == first)) {
                edge_exists = true;
                break;
            }
        }
        // Если такого ребра еще нет, то добавляем его
        if (!edge_exists) {
            edges.push_back(std::make_tuple(first, second, weight)); // Добавление ребра в список рёбер
            adj[first].push_back(second); // Добавление второй вершины в список смежности первой
            adj[second].push_back(first); // Добавление первой вершины в список смежности второй
        }
    }

    void display() { // Метод для отображения графа (соседние элементы: номер вершины: номера соседних)
        for(int i = 0; i < n; i++) { // Цикл по всем вершинам графа
            std::cout << i << ": "; // Вывод индекса вершины
            for(auto x: adj[i]) { // Цикл по списку смежности текущей вершины
                std::cout << x << " "; // Вывод вершин, смежных с текущей
            }
            std::cout << "\n"; // Переход на новую строку для следующей вершины
        }
    }

    void print_edges() { // Метод для вывода списка рёбер
        std::cout << "List of edges: ";
        for(auto x : edge) { // Цикл по всем рёбрам в векторе рёбер
            std::cout << "[" << x.first << ", " << x.second << "] "; // Вывод текущего ребра в формате (вершина1, вершина2)
        }
        std::cout << std::endl; // Переход на новую строку после вывода всех рёбер
    }

    void print_edges_with_weights() { // Метод для вывода списка рёбер + их значения
        std::cout << "List of edges with weights: " << std::endl;
        std::set<std::pair<int, int>> printed_edges; // Множество для отслеживания уже выведенных рёбер
        for (auto u : edges) {
            int first = std::get<0>(u); // std::get<значение>(название кортежа) - получение значение под некотором от некоторого кортежа
            int second = std::get<1>(u);
            int weight = std::get<2>(u);
            // Проверяем, не вывели ли уже ребро в обратном направлении
            if (printed_edges.find({second, first}) == printed_edges.end()) {
                std::cout << "{[" << first << " , " << second << "] "  << weight << "}  ";
                printed_edges.insert({first, second}); // Добавляем ребро в множество выведенных
            }
        }
        std::cout << std::endl;
    }

    int what_type_of_edges () { // Метод проверки типа ребра (нужно для вывода списка ребер)
        if (edge.empty()) { // Если список с обычными ребрами пустой, следовательно, это ребра со значением
            return 2; // Тогда вернем двойку
        }
        // Если список с обычными ребрами непустой, следовательно, это и есть обычные ребра
        return 1; // Тогда и вернем единицу
    }

    void print_adjacency_matrix() {
        // Выводим номерацию вершин в первой строке
        std::cout << "   ";
        for (int i = 0; i < n; ++i) { // Выведем первую строчку с нумерацией вершин
            std::cout << i << " ";
        }
        std::cout << std::endl;
        // Выводим матрицу смежности
        for (int i = 0; i < n; i++) {
            std::cout << i << ": "; // Выводим номер строки (вершины)
            for (int j = 0; j < n; j++) {
                bool is_adjacent = false; // Для проверки наличия ребра между двумя вершинами
                                          // Изначально мы утверждаем отсутствие связи
                for (auto k : adj[i]) { // Проверяем наличия ребра между вершиной i и j
                    if (k == j) { // Если ребро есть
                        is_adjacent = true; // Если ребро есть
                        break;
                    }
                }
                if (is_adjacent) {
                    std::cout << "1 ";
                } else {
                    std::cout << "0 ";
                }
            }
            std::cout << std::endl;
        }
    }

    std::vector<int> DFS(int start_vertex, std::vector<bool>& visited) { // Метод для выполнения поиска в глубину (DFS)
        visited[start_vertex] = true; // Помечаем текущую вершину как посещённую
        std::vector<int> dfs_result; // Вектор для хранения результатов DFS
        for (auto u : adj[start_vertex]) { // Идём по всем смежным вершинам
            if (!visited[u]) { // Если вершина не посещена
                std::vector<int> sub_dfs_result = DFS(u, visited); // Рекурсивно выполняем DFS от неё
                dfs_result.insert(dfs_result.end(),
                                  sub_dfs_result.begin(), sub_dfs_result.end()); // Добавляем результаты под-DFS в общий вектор
            }
        }
        dfs_result.push_back(start_vertex); // Добавляем текущую вершину в результат DFS
        return dfs_result; // Возвращаем результат DFS
    }

    void print_DFS(int start_vertex) { // Метод для вывода результатов поиска в глубину (DFS)
        std::vector<bool> visited(n, false); // Вектор для отслеживания посещенных вершин
        std::cout << "DFS from vertex " << start_vertex << ": ";
        std::vector<int> dfs_result = DFS(start_vertex, visited);
        for (auto v : dfs_result) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    std::vector<int> BFS(int start_vertex, std::vector<bool>& visited) { // Метод для выполнения поиска в ширину (BFS)
        std::vector<int> bfs_result; // Вектор для хранения результатов BFS
        std::queue<int> q; // Очередь для выполнения поиска в ширину
        visited[start_vertex] = true; // Помечаем начальную вершину как посещенную
        q.push(start_vertex); // Добавляем начальную вершину в очередь
        while (!q.empty()) { // Пока очередь не пуста
            int v = q.front(); // Получаем вершину из передней части очереди
            q.pop(); // Удаляем вершину из очереди
            bfs_result.push_back(v); // Добавляем текущую вершину в результат BFS
            // Идем по всем смежным вершинам текущей вершины
            for (auto u : adj[v]) {
                if (!visited[u]) { // Если вершина не посещена
                    visited[u] = true; // Помечаем её как посещенную
                    q.push(u); // Добавляем её в очередь
                }
            }
        }
        return bfs_result; // Возвращаем результат BFS
    }

    void print_BFS(int start_vertex) { // Метод для вывода результатов поиска в ширину (BFS)
        std::vector<bool> visited(n, false); // Вектор для отслеживания посещенных вершин
        std::vector<int> bfs_result = BFS(start_vertex, visited); // Выполняем поиск в ширину
        std::cout << "BFS from vertex " << start_vertex << ": ";
        for (auto v : bfs_result) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    bool is_connected() { // Метод для проверки связности графа
        std::vector<bool> visited(n, false); // Вектор для отслеживания посещенных вершин
        DFS(0, visited); // Выполняем DFS от начальной вершины
        for (int i = 0; i < visited.size(); i++) {
            if (!visited[i]) return false;
        }
        return true;
    }

    void connected_components() { // Метод для вывода компонентов связности
        std::vector<bool> visited(n, false); // Вектор для отслеживания посещенных вершин
        int component_count = 0; // Счётчик компонент связности
        for (int i = 0; i < n; i++) {
            if (!visited[i]) { // Если вершина не посещена, значит, это новая компонента связности
                component_count++;
                std::cout << "Component number " << component_count << ": ";
                std::vector<int> dfs_result = DFS(i, visited); // Выполняем DFS от данной вершины
                for (auto v : dfs_result) {
                    std::cout << v << " ";
                }
                std::cout << std::endl;
            }
        }
        std::cout << "Quantity connected components: " << component_count << std::endl;
    }

    bool has_cycle() {
        std::vector<bool> visited(n, false); // Вектор для отслеживания посещенных вершин
        for (int i = 0; i < n; i++) { // parent - предыдущая вершина для каждой последующей вершины, изначально равная -1,
                                      // потому что мы отдельно рассматриваем каждую вершину
                                      // При рассмотрении последующих вершин относительно передаваемой, parent будет меняться
            if (!visited[i] && DFS_has_cycle(i, -1, visited)) // Запускаем DFS для каждой вершины
                return true; // Если нашли цикл, возвращаем true
        }
        return false; // Если не нашли цикл после обхода всех вершин, возвращаем false
    }

    bool DFS_has_cycle(int vertex, int parent, std::vector<bool>& visited) {
        visited[vertex] = true; // Помечаем текущую вершину как посещенную
        for (auto u : adj[vertex]) {
            if (!visited[u]) { // Если вершина не посещена
                if (DFS_has_cycle(u, vertex, visited)) // Запускаем DFS для нее
                    return true; // Если найден цикл, возвращаем true
            } else if (u != parent) { // Если вершина посещена и не является родителем текущей
                return true; // Нашли цикл, возвращаем true
            }
        }
        return false; // Если не нашли цикл при обходе смежных вершин, возвращаем false
    }

    bool is_bipartite() {
        std::vector<int> colors(n, -1); // Вектор для хранения цветов вершин
        std::queue<int> q; // Очередь для выполнения обхода в ширину
        // Выбираем начальную вершину и присваиваем ей цвет 0
        q.push(0);
        colors[0] = 0;
        while (!q.empty()) {
            int u = q.front(); // Получаем вершину из очереди
            q.pop(); // Удаляем вершину из очереди
            // Перебираем всех соседей вершины u
            for (int v : adj[u]) {
                // Если вершина v еще не окрашена
                if (colors[v] == -1) {
                    // Окрашиваем вершину v в противоположный цвет от вершины u
                    colors[v] = 1 - colors[u];
                    // Добавляем вершину v в очередь для обработки её соседей
                    q.push(v);
                }
                    // Если вершина v уже окрашена и её цвет совпадает с цветом вершины u
                else if (colors[v] == colors[u]) {
                    return false; // Граф не является двудольным
                }
            }
        }
        return true; // Граф является двудольным
    }

    void print_shortest_distances(std::vector<int>& distances, int start_vertex) { // Метод для вывода кратчайших расстояний
        std::cout << "Shortest distances from vertex " << start_vertex << ":\n";
        for (int i = 0; i < n; i++) { // Начинаем вывод не от начальной вершины, так как расстояние от нее до самой себя равно 0
            std::cout << "Vertex " << i << ": ";
            if (distances[i] == INT_MAX) {  // Проверяем, достижима ли текущая вершина из начальной
                // Если кратчайшее расстояние до вершины равно INT_MAX,
                // то вершина недостижима из начальной (неизвестное расстояние)
                // В противном случае выводится кратчайшее расстояние
                std::cout << "There is no way\n";
            } else {
                std::cout << distances[i] << std::endl;
            }
        }
    }

    std::vector<int> bellman_ford_shortest_paths(int start_vertex) {
        // Инициализируем вектор расстояний, начальное расстояние от всех вершин до start_vertex равно бесконечности
        std::vector<int> distance(n, INT_MAX);
        // Расстояние от start_vertex до самого себя равно 0
        distance[start_vertex] = 0;
        // Учитываем каждое ребро дважды - в каждом направлении
        for (int i = 0; i < n - 1; i++) {
            for (auto u : edges) {
                // Получаем данные о ребре
                int first = std::get<0>(u);
                int second = std::get<1>(u);
                int weight = std::get<2>(u);
                // Обновляем расстояние до вершины second, если найден более короткий путь через first
                if (distance[first] != INT_MAX && distance[first] + weight < distance[second]) {
                    distance[second] = distance[first] + weight;
                }
                // Теперь учитываем ребро в обратном направлении
                if (distance[second] != INT_MAX && distance[second] + weight < distance[first]) {
                    distance[first] = distance[second] + weight;
                }
            }
        }
        return distance; // Возвращаем вектор расстояний от start_vertex до всех остальных вершин
    }

    std::vector<int> dijkstra_shortest_paths(int start_vertex) {
        std::vector<int> distance(n, INT_MAX); // Инициализация расстояний.
        // Изначально все элементы этого вектора устанавливаются равными INT_MAX,
        // что соответствует бесконечности. Это делается для обозначения того,
        // что изначально не известно ни одно расстояние от начальной вершины
        // до других вершин графа
        distance[start_vertex] = 0; // Расстояние от начальной вершины до самой себя равно 0
        // Создаем приоритетную очередь (в которой первый элемент пары представляет расстояние
        // от начальной вершины до данной вершины, а второй элемент пары представляет саму вершину),
        // где вершины будут отсортированы по их расстоянию
        // от начальной вершины. Используем std::greater, чтобы сделать приоритетную очередь
        // мин-кучей, чтобы вершины с меньшим расстоянием находились в верхней части очереди.
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
        pq.push({0, start_vertex}); // Добавление стартовой вершины в очередь с расстоянием 0.
        // Основной цикл алгоритма Дейкстры.
        while (!pq.empty()) {
            // Извлечение вершины с наименьшим расстоянием из очереди.
            int x = pq.top().second; // Получение вершины.
            pq.pop(); // Удаление вершины из очереди.
            // Проход по всем рёбрам графа.
            for (auto u : edges) {
                int first = std::get<0>(u); // Получение первой вершины ребра.
                int second = std::get<1>(u); // Получение второй вершины ребра.
                int weight = std::get<2>(u); // Получение веса ребра.
                // Если данное ребро связывает эту вершину с другой вершиной
                if (first == x) {
                    // Попытка обновить расстояние до вершины to.
                    if (distance[x] + weight < distance[second]) {
                        // Если новое расстояние меньше текущего, обновление его и добавление вершины to в очередь.
                        distance[second] = distance[x] + weight;
                        pq.push({distance[second], second});
                    }
                }
                // Обработка рёбер, связанных с вершиной to.
                else if (second == x) {
                    if (distance[x] + weight < distance[first]) {
                        distance[first] = distance[x] + weight;
                        pq.push({distance[first], first});
                    }
                }
            }
        }
        // Возвращение вектора, содержащего кратчайшие расстояния от начальной вершины до всех остальных вершин в графе
        return distance;
    }

    std::vector<std::vector<int>> floyd_warshall_shortest_paths() {
        std::vector<std::vector<int>> distance(n, std::vector<int>(n, INT_MAX));
        // Устанавливаем расстояние от каждой вершины до самой себя равным 0.
        for (int i = 0; i < n; ++i) {
            distance[i][i] = 0;
        }
        // Заполнение матрицы расстояний известными рёбрами графа.
        for (auto u : edges) {
            int first = std::get<0>(u);
            int second = std::get<1>(u);
            int weight = std::get<2>(u);
            distance[first][second] = weight;
            distance[second][first] = weight; // Учитываем обратное ребро, если граф неориентированный
        }
        // Цикл по всем парам вершин графа
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    // Проверяем, существует ли более короткий путь из вершины i в вершину j через вершину k.
                    if (distance[i][k] != INT_MAX && distance[k][j] != INT_MAX && distance[i][k] + distance[k][j] < distance[i][j]) {
                        // Если нашли более короткий путь, обновляем значение расстояния между i и j.
                        distance[i][j] = distance[i][k] + distance[k][j];
                    }
                }
            }
        }
        // Возвращаем матрицу расстояний, содержащую кратчайшие расстояния между всеми парами вершин.
        return distance;
    }

    ~graph() {
        // Очищаем список рёбер
        edge.clear();
        edges.clear();
        // Очищаем список смежности
        for (auto& adj_list : adj) {
            adj_list.clear();
        }
    }
};

#endif //GRAPH_GRAPH_H
