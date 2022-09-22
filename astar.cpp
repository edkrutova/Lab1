#include "astar.h"
#include "dijkstra.h"

SearchResult AStar::findPath(Input input) //������ ����� � ������� A*
{
    auto t = std::chrono::high_resolution_clock::now(); //��������� ������� �����
    SearchResult result; //������� ������ ������ ��� ������
    input.start.f=0; // ������ ��������1 ��� ��������� ������� (���������� �� ��������� ������� �� �������) 
    input.start.g = getHValue(input.start, input.goal, input.map.diagonal_moves_allowed); 
    // ��������2 ��� ��������� ������� (���������� �� ������� ������� �� �������� �� ������)
    open.addNode(input.start); //��������� ��������� ������� � ������ ��� ���������

    while (open.getSize()) { //���� ������ ��������� �� ����
        auto current = open.getMin(); //����� ������ ������� �� ������ ���������(�� ����� ������)
        closed.addClose(current);//��������� � ������ ������������ �����, ���� �� ������ � ����� ������� ��������� �� ����
        open.popMin(); // ������� ���� �������, ���� �� �������
        auto tol = input.map.getValidMoves(current); // �������� ������ ��������� ������ ��� �������
        for (auto b: tol) { //���������� ���� ������� ��� ������� �������
            if (!closed.inClose(b.x, b.y)) { //���� ������� ��� �� ����������
                b.f = input.map.getCost(current, b) + current.f ;
                // ��������� ��������1 ������� ������ 
                b.g = getHValue(b,input.goal, input.map.diagonal_moves_allowed);
                // ��������� ��������2 ������� ������
               
                b.parent = closed.getPointer(current.x, current.y); // �������� ��������� �� ��������
                open.addNode(b); // � ������ �� ��������� ����������� ������� �����
            }
        }
        if (current.x == input.goal.x && current.y == input.goal.y) { //���� ������� ������� �������� ��������
            result.pathfound = true; //���� ������
            auto answer = reconstructPath(current); //������������ ���� � ��������� ���������� �����
            result.path = answer.first; //���������� ���� � ���������
            result.steps = answer.second; //���������� ���������� ����� � ���������
            result.cost = current.f; //���������� ���������
            break;
        }
    }

    result.createdNodes = closed.getSize() + open.getSize(); //��������� ���������� ��������� ������
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count();
    // ��������� ����� ������
    return result; //���������� ���������
}
double AStar::getHValue(Node current, Node goal, bool dma) // ��������� ���������� �� ����� �� �������� �� ������
{
    if (dma) //���� ����� ������ �� ���������

        return sqrt(pow((goal.x - current.x),2) + pow((goal.y-current.y),2)); //��������� ��������� ���������� ����� �������
    return abs(goal.x - current.x)+ abs(goal.y-current.y); //��������� ������������� ���������� ����� �������
}

std::pair<std::list<Node>, int> AStar::reconstructPath(Node current) //�������������� ����, ������� ����� �����
{
    std::list<Node> path; //������� ������ ����� ��� �������� ����
    int Stepscount = 0;
    while(current.parent != nullptr) // ���� ������� ������ �� ������ ���������(�� � ������� ��������)
    {
        path.push_front(current); //��������� ������� ������� � ������ ������
        current = *current.parent; // ������ �������� ������� ���������
        ++Stepscount;
    }
    path.push_front(current); //��������� ��������� ������� ��������
    return std::make_pair(path, Stepscount);  //���������� ������ ����� ������� :)
}

