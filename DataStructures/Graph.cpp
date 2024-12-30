#include "Graph.h"

template<>
DynamicArraySmart<ShrdPtr<Edge<std::string>>> Dijkstra::findShortestPath(const ShrdPtr<Node<std::string>>& startNode, const ShrdPtr<Node<std::string>>& targetNode) {
    using NodeDist = std::pair<int, ShrdPtr<Node<std::string>>>;
    auto cmp = [](const NodeDist& left, const NodeDist& right) { return left.first > right.first; };
    std::priority_queue<NodeDist, std::vector<NodeDist>, decltype(cmp)> pq(cmp);

    HashTable<ShrdPtr<Node<std::string>>, int> distances;
    HashTable<ShrdPtr<Node<std::string>>, ShrdPtr<Edge<std::string>>> previousEdge;

    distances.Add(startNode, 0);
    pq.push({0, startNode});

    while (!pq.empty()) {
        auto [currentDistance, currentNode] = pq.top();
        pq.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (const auto& edge : currentNode->getEdges()) {
            ShrdPtr<Node<std::string>> neighbor = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
            int newDistance = currentDistance + edge->getWeight();

            if (!distances.ContainsKey(neighbor) || newDistance < distances.Get(neighbor)) {
                if (distances.ContainsKey(neighbor)) {
                    distances.Update(neighbor, newDistance);
                } else {
                    distances.Add(neighbor, newDistance);
                }

                if (previousEdge.ContainsKey(neighbor)) {
                    previousEdge.Update(neighbor, edge);
                } else {
                    previousEdge.Add(neighbor, edge);
                }

                pq.push({newDistance, neighbor});
            }
        }
    }

    DynamicArraySmart<ShrdPtr<Edge<std::string>>> shortestPath;
    ShrdPtr<Node<std::string>> currentNode = targetNode;

    while (currentNode != startNode) {
        if (!previousEdge.ContainsKey(currentNode)) {
            return DynamicArraySmart<ShrdPtr<Edge<std::string>>>();
        }

        ShrdPtr<Edge<std::string>> edge = previousEdge.Get(currentNode);
        shortestPath.Prepend(edge);
        currentNode = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
    }

    return shortestPath;
}

template<>
DynamicArraySmart<ShrdPtr<Edge<std::string>>> BellmanFord::findShortestPath(
    const ShrdPtr<Node<std::string>>& startNode,
    const ShrdPtr<Node<std::string>>& targetNode,
    const DynamicArraySmart<ShrdPtr<Edge<std::string>>>& edges,
    const DynamicArraySmart<ShrdPtr<Node<std::string>>>& nodes)
{
    HashTable<ShrdPtr<Node<std::string>>, int> distances;
    HashTable<ShrdPtr<Node<std::string>>, ShrdPtr<Edge<std::string>>> previousEdge;

    for(int i = 0; i < nodes.GetLength(); ++i) {
        distances.Add(nodes[i], std::numeric_limits<int>::max());
    }
    distances.Update(startNode, 0);

    for (size_t i = 0; i < nodes.GetLength() - 1; ++i) {
        for (int j = 0; j < edges.GetLength(); ++j) {
            const auto& edge = edges[j];
            ShrdPtr<Node<std::string>> fromNode = edge->getFromNode();
            ShrdPtr<Node<std::string>> toNode = edge->getToNode();
            int weight = edge->getWeight();

            if (distances.Get(fromNode) != std::numeric_limits<int>::max() &&
                    distances.Get(fromNode) + weight < distances.Get(toNode)) {
                distances.Update(toNode, distances.Get(fromNode) + weight);

                if (previousEdge.ContainsKey(toNode)) {
                    previousEdge.Update(toNode, edge);
                } else {
                    previousEdge.Add(toNode, edge);
                }
            }
        }
    }

    for (int j = 0; j < edges.GetLength(); ++j) {
        const auto& edge = edges[j];
        ShrdPtr<Node<std::string>> fromNode = edge->getFromNode();
        ShrdPtr<Node<std::string>> toNode = edge->getToNode();
        int weight = edge->getWeight();


        if (distances.Get(fromNode) != std::numeric_limits<int>::max() &&
            distances.Get(fromNode) + weight < distances.Get(toNode)) {
            throw std::runtime_error("Graph contains a negative-weight cycle");
            }
    }

    DynamicArraySmart<ShrdPtr<Edge<std::string>>> shortestPath;
    ShrdPtr<Node<std::string>> currentNode = targetNode;

    while (currentNode != startNode) {
        if (!previousEdge.ContainsKey(currentNode)) {
            return DynamicArraySmart<ShrdPtr<Edge<std::string>>>();
        }

        ShrdPtr<Edge<std::string>> edge = previousEdge.Get(currentNode);
        shortestPath.Prepend(edge);
        currentNode = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
    }

    return shortestPath;
}
