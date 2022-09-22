#include "dijkstra.h"

SearchResult Dijkstra::findPath(Input input) 
{
    auto t = std::chrono::high_resolution_clock::now(); //�������� ����� ������ ������
    SearchResult result;   // ������� ������ ���������
    input.start.f=0;  // �������� ������ ������� ������ (0)
    open.addNode(input.start); // �������� ��������� ������� � ������ �� ���������

    while (open.getSize()) {   
        auto current = open.getMin();  //����� ������ ������� �� ������ ��������� (�� ����� ������)
        closed.addClose(current); //��������� � ������ ������������ �����, ���� �� ������ � ����� ������� ��������� �� ����
        open.popMin(); // ������� ���� �������, ���� �� �������
        auto tol = input.map.getValidMoves(current); // �������� ������ ��������� ������ ��� �������
        for (auto b: tol) {  //���������� ���� ������� ��� ������� �������
            if (!closed.inClose(b.x, b.y)) {  //���� ������� ��� �� ����������
                b.f = input.map.getCost(current, b) + current.f;  // ��������� �������� ������� ������ 
                b.parent = closed.getPointer(current.x, current.y); // �������� ��������� �� ��������
                open.addNode(b);  // � ������ �� ��������� ����������� ������� �����
            }
        }
        if (current.x == input.goal.x && current.y == input.goal.y) {
            //closed.addClose(current); // ?
            result.pathfound = true;  // ������� �����!
            auto answer = reconstructPath(current);
            result.path = answer.first;
            result.steps = answer.second;
            result.cost = current.f; // ������� �������� �������� �������� �������
            break;
        }
    }

    result.createdNodes = closed.getSize() + open.getSize(); //����� ���� ����� ������� ������ = �������� ���������� ��������� ������
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count(); //����� ������ - ������� ����� ���������
    return result;
}

std::pair<std::list<Node>, int> Dijkstra::reconstructPath(Node current) //�������������� ����, ������� ����� �����
{
    std::list<Node> path;  //������� ������ ����� ��� �������� ����
    int stepsCount = 0;
    while(current.parent != nullptr) // ���� ������� ������ �� ������ ���������(�� � ������� ��������)
    {
        path.push_front(current); //��������� ������� ������� � ������ ������
        current = *current.parent; // ������ �������� ������� ���������
        ++stepsCount;
    }
    path.push_front(current); //��������� ��������� ������� ��������
    return make_pair(path, stepsCount); //���������� ������ ����� ������� :)
}
