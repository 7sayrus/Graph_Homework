#include "Graph.h"
#include <iostream>
using namespace std;

void showGraph(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    cout << "Текущий список смежности графа:" << endl;
    graph.printGraph();
    cout << "-" << string(28, '-') << endl;
}

void addNodeInGraph(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    string nameNode;
    int countNode;

    cout << "Введите название вершины: ";
    cin >> nameNode;

    while (true) {
        cout << "Введите количество участников вершины: ";
        cin >> countNode;
        if (!cin || countNode <= 0) {
            cerr << "Ошибка ввода! Введите положительное целое число." << endl;
        } else {
            break;
        }
    }

    graph.addNode(nameNode, countNode);
    cout << "-" << string(28, '-') << endl;
}

void addEdgeInGraph(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    string nameNode1, nameNode2;

    while (true) {
        cout << "Введите название первой вершины: ";
        cin >> nameNode1;
        if (graph.InGraph(nameNode1)) {
            break;
        } else {
            cerr << "Такой вершины нет, попробуйте еще раз." << endl;
        }
    }

    while (true) {
        cout << "Введите название второй вершины: ";
        cin >> nameNode2;
        if (graph.InGraph(nameNode2)) {
            break;
        } else {
            cerr << "Такой вершины нет, попробуйте еще раз." << endl;
        }
    }

    graph.addEdges(nameNode1, nameNode2);
    cout << "-" << string(28, '-') << endl;
}

void bfs(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    cout << "Обход в ширину" << endl;
    string startNode;

    while (true) {
        cout << "Введите название вершины: ";
        cin >> startNode;
        if (graph.InGraph(startNode)) {
            break;
        } else {
            cerr << "Такой вершины нет, попробуйте еще раз." << endl;
        }
    }

    vector<string> result = graph.bfs(startNode);
    for (const auto& node : result) {
        cout << node << " ";
    }
    cout << endl;
    cout << "-" << string(28, '-') << endl;
}

void dfs(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    cout << "Обход в глубину" << endl;
    string startNode;

    while (true) {
        cout << "Введите название вершины: ";
        cin >> startNode;
        if (graph.InGraph(startNode)) {
            break;
        } else {
            cerr << "Такой вершины нет, попробуйте еще раз." << endl;
        }
    }

    set<string> visited;
    vector<string> result = graph.dfs(startNode, visited);
    for (const auto& node : result) {
        cout << node << " ";
    }
    cout << endl;
    cout << "-" << string(28, '-') << endl;
}

void restructurePlan(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    cout << "Переработанный список смежности графа:" << endl;
    Graph newGraph = graph.restructurePlan();
    newGraph.printGraph();
    cout << "-" << string(28, '-') << endl;
}

void menu(Graph &graph) {
    cout << "-" << string(28, '-') << endl;
    cout << "Выберите действие:\n"
         << "1. Добавить вершину\n"
         << "2. Добавить ребро\n"
         << "3. Показать список смежности графа\n"
         << "4. Обход графа в ширину\n"
         << "5. Обход графа в глубину\n"
         << "6. Переработать граф с учетом плана\n"
         << "0. Завершить работу программы\n";

    int choice;
    cin >> choice;

    switch (choice) {
    case 0:
        return;
    case 1:
        addNodeInGraph(graph);
        menu(graph);
        break;
    case 2:
        addEdgeInGraph(graph);
        menu(graph);
        break;
    case 3:
        showGraph(graph);
        menu(graph);
        break;
    case 4:
        bfs(graph);
        menu(graph);
        break;
    case 5:
        dfs(graph);
        menu(graph);
        break;
    case 6:
        restructurePlan(graph);
        menu(graph);
        break;
    default:
        cerr << "Неверный выбор. Попробуйте снова.\n";
        menu(graph);
        break;
    }
}
#include <codecvt>
int main() {
    setlocale(LC_ALL, "ru_RU.UTF-8");
    Graph graph;
    graph.graph = {
        { "A", { "B", "C" } },
        { "B", { "A" } },
        { "C", { "B" } }
    };
    graph.count = {
        { "A", 10 },
        { "B", 20 },
        { "C", 5 }
    };

    menu(graph);

    return 0;
}