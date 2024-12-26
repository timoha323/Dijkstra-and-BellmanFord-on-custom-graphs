#include "Graph.h"

int Node::nodeId_ = 0;

Node::Node() {
    name_ = std::to_string(nodeId_++);
}

Node::Node(std::string name) : name_(std::move(name)) {
    nodeId_++;
}

const std::string Node::getName() const {
    return name_;
}

Node::Node(Node&& other) noexcept : edges_(std::move(other.edges_)), name_(std::move(other.name_)) {}

Node& Node::operator=(Node&& other) noexcept {
    if (this != &other) {
        edges_ = std::move(other.edges_);
    }
    return *this;
}

void Node::addEdge(const ShrdPtr<Edge>& edge) {
    edges_.Append(edge);
}

const LinkedListSmart<ShrdPtr<Edge>>& Node::getEdges() const {
    return edges_;
}

Edge::Edge(int weight, const ShrdPtr<Node>& fromNode, const ShrdPtr<Node>& toNode)
    : weight_(weight), fromNode_(fromNode), toNode_(toNode) {}

int Edge::getWeight() const {
    return weight_;
}

ShrdPtr<Node> Edge::getFromNode() const {
    return fromNode_;
}

ShrdPtr<Node> Edge::getToNode() const {
    return toNode_;
}


ShrdPtr<Node> Graph::createNode() {
    auto node = ShrdPtr<Node>(new Node());
    nodes_.Append(node);
    return node;
}
ShrdPtr<Node> Graph::getNodeByName(const std::string& name) {
    for (int i = 0; i < nodes_.GetLength(); ++i) {
        auto node = nodes_[i];
        if (node->getName() == name) {
            return node;
        }
    }
    throw std::invalid_argument("Node does not exist");
}

ShrdPtr<Node> Graph::createNode(const std::string& nodeName) {
    auto node = ShrdPtr<Node>(new Node(nodeName));
    nodes_.Append(node);
    return node;
}

ShrdPtr<Edge> Graph::createEdge(int weight, const ShrdPtr<Node>& fromNode, const ShrdPtr<Node>& toNode) {
    auto edge = ShrdPtr<Edge>(new Edge(weight, fromNode, toNode));
    edges_.Append(edge);
    fromNode->addEdge(edge);
    toNode->addEdge(edge);
    return edge;
}

const DynamicArraySmart<ShrdPtr<Node>>& Graph::getNodes() const {
    return nodes_;
}

const DynamicArraySmart<ShrdPtr<Edge>>& Graph::getEdges() const {
    return edges_;
}

DynamicArraySmart<ShrdPtr<Edge>> Dijkstra::findShortestPath(const ShrdPtr<Node>& startNode, const ShrdPtr<Node>& targetNode) {
    using NodeDist = std::pair<int, ShrdPtr<Node>>;
    auto cmp = [](const NodeDist& left, const NodeDist& right) { return left.first > right.first; };
    std::priority_queue<NodeDist, std::vector<NodeDist>, decltype(cmp)> pq(cmp);

    HashTable<ShrdPtr<Node>, int> distances;
    HashTable<ShrdPtr<Node>, ShrdPtr<Edge>> previousEdge;

    distances.Add(startNode, 0);
    pq.push({0, startNode});

    while (!pq.empty()) {
        auto [currentDistance, currentNode] = pq.top();
        pq.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (const auto& edge : currentNode->getEdges()) {
            ShrdPtr<Node> neighbor = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
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

    DynamicArraySmart<ShrdPtr<Edge>> shortestPath;
    ShrdPtr<Node> currentNode = targetNode;

    while (currentNode != startNode) {
        if (!previousEdge.ContainsKey(currentNode)) {
            return DynamicArraySmart<ShrdPtr<Edge>>();
        }

        ShrdPtr<Edge> edge = previousEdge.Get(currentNode);
        shortestPath.Prepend(edge);
        currentNode = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
    }

    return shortestPath;
}

DynamicArraySmart<ShrdPtr<Edge>> BellmanFord::findShortestPath(
    const ShrdPtr<Node>& startNode,
    const ShrdPtr<Node>& targetNode,
    const DynamicArraySmart<ShrdPtr<Edge>>& edges,
    const DynamicArraySmart<ShrdPtr<Node>>& nodes)
{
    HashTable<ShrdPtr<Node>, int> distances;
    HashTable<ShrdPtr<Node>, ShrdPtr<Edge>> previousEdge;

    for(int i = 0; i < nodes.GetLength(); ++i) {
        distances.Add(nodes[i], std::numeric_limits<int>::max());
    }
    distances.Update(startNode, 0);

    for (size_t i = 0; i < nodes.GetLength() - 1; ++i) {
        for (int j = 0; j < edges.GetLength(); ++j) {
            const auto& edge = edges[j];
            ShrdPtr<Node> fromNode = edge->getFromNode();
            ShrdPtr<Node> toNode = edge->getToNode();
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
        ShrdPtr<Node> fromNode = edge->getFromNode();
        ShrdPtr<Node> toNode = edge->getToNode();
        int weight = edge->getWeight();

        if (distances.Get(fromNode) != std::numeric_limits<int>::max() &&
            distances.Get(fromNode) + weight < distances.Get(toNode)) {
            throw std::runtime_error("Graph contains a negative-weight cycle");
        }
    }

    DynamicArraySmart<ShrdPtr<Edge>> shortestPath;
    ShrdPtr<Node> currentNode = targetNode;

    while (currentNode != startNode) {
        if (!previousEdge.ContainsKey(currentNode)) {
            return DynamicArraySmart<ShrdPtr<Edge>>();
        }

        ShrdPtr<Edge> edge = previousEdge.Get(currentNode);
        shortestPath.Prepend(edge);
        currentNode = (edge->getFromNode() == currentNode) ? edge->getToNode() : edge->getFromNode();
    }

    return shortestPath;
}
