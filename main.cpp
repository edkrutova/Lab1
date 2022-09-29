#include <iostream>
#include <set>
#include <cmath>
#include "structs.h"
#include "utils.h"
#include <chrono>


class BFS //breadth-first-search
{
public:
    Result find_path(Node start, Node goal, Map grid)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0; // g - вес ребра, если что
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound)
        {
           Node current = OPEN.front();
           OPEN.pop_front();
           steps++;
           auto neighbors = grid.get_neighbors(current,4);
           for(auto n:neighbors) {
               if (CLOSED.find(n) == CLOSED.end())
               {
                   n.g = current.g + 1;
                   n.parent = &(*CLOSED.find(current));
                   OPEN.push_back(n);
                   CLOSED.insert(n);
                   if(n == goal) {
                       result.path = reconstruct_path(n);
                       result.cost = n.g;
                       pathfound = true;
                       break;
                    }
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar {
public:
    Result find_path(Node start, Node goal, Map grid, std::string metrictype="Manhattan", int connections=4, double hweight=1) { //создаем метод find path, имеет значения по умолч,
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result; //объект result класса Result
        int steps = 0;
        start.g = 0;
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound) {
            Node current = OPEN.front();
            // назначить вершину с наименьшим значением f как current (текущую)
            for (auto n: OPEN) {
                if (n.f < current.f) //f = g + w*h нужно чтобы оценивать стоимость
                    current = n;
            }
            OPEN.remove(current);
            steps++;
            auto neighbors = grid.get_neighbors(current, connections);
            CLOSED.insert(current);
            for (auto n: neighbors) {
                if (CLOSED.find(n) == CLOSED.end()) { //если в closed нет n (соседа)
                    if (n.i != current.i && n.j != current.j) //если i и j не совпадают, то был переход по диаг.
                        n.g = current.g + 1.41421;
                    else
                        n.g = current.g + 1;
                    n.h = hweight * count_h_value(n, goal, metrictype); //оценивает от тек.соседа до цели с нужной эвристикой + hweight
                    n.f = n.g + n.h; // используем формулу f = g + h*w
                    n.parent = &(*CLOSED.find(current)); //указываем родителя соседа - это тек. вершина (из списка  closed)
                    OPEN.push_back(n); //добавляем соседа в open в конец
                    if (n == goal) {
                        result.path = reconstruct_path(n);
                        result.cost = n.g;
                        pathfound = true;
                        break;
                    }
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }
    double count_h_value(Node current, Node goal, std::string metrictype="Octile") {
        double answer;
        double dx = abs(current.i - goal.i);
        double dy = abs(current.j - goal.j);
        int heuristic_type = -1;

        if (metrictype == "Euclidean") heuristic_type = 0;
        else if (metrictype == "Manhattan") heuristic_type = 1;
        else if (metrictype == "Octile") heuristic_type = 2;

        switch (heuristic_type) {
            case 0:
                // Euclidean distance
                answer = sqrt(pow(current.i - goal.i, 2) + pow(current.j - goal.j, 2));
                break;
            case 1:
                // Manhattan distance
                answer = abs(current.i - goal.i) + abs(current.j - goal.j);
                break;
            case 2:
                // Ocitle (diagonal) distance
                answer = abs(dx - dy) + sqrt(2) * std::min(dx, dy);
                break;
            default:
                answer = 0;
        }
        return answer;
    }
    std::list<Node> reconstruct_path(Node m) {
        std::list<Node> path;
        while(m.parent != nullptr) {
            path.push_front(m);
            m = *m.parent;
        }
        path.push_front(m);
        return path;
    }
};

int main(int argc, char* argv[]) //argc - argumnet counter, argv - argument values
{
    
    for(int i=0; i<argc; i++) //argc - размер массива argv (сколько аргументов(название например) передано)
        std::cout<<argv[i]<<"\n";
    if(argc<2)
    {
        std::cout << "Name of the input XML file is not specified."<<std::endl;
        return 1;
    }
    Loader loader; //загрузка графа объект loader класса Loader
    loader.load_instance(argv[1]);
    Result result;
    if(loader.algorithm == "BFS")
    {
        std::cout << "This is BFS" << std::endl;
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid);
    }

    else if (loader.algorithm == "Dijkstra") {
        std::cout << "This is Dijkstra" << std::endl;
        loader.hweight = 0;
        AStar d;
        result = d.find_path(loader.start, loader.goal, loader.grid);
    }
    else  if (loader.algorithm == "AStar") {
        if (loader.metrictype == "Manhattan" && loader.connections == 8 ) {
            std::cout<<"Manhattan distance is an inadmissible heuristic when diagonal moves are allowed"<<std::endl;
            return 0;
        }
        std::cout << "This is AStar" << std::endl;
        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
    }
    loader.grid.print(result.path);
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
    <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;
    return 0;
}