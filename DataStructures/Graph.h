#pragma once
#include "LinkedListSmart.h"
#include "ShrdPtr.h"
#include "HashTable.h"

#include <functional>
#include <queue>
#include <utility>
#include <limits>
#include <stdexcept>

class Node {
private:
    LinkedListSmart<ShrdPtr<Edge>> edges_;
    std::string name_;
    static int nodeId_;

public:
    Node();
    Node(std::string name);

    const std::string getName() const;

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    Node(Node&& other) noexcept;
    Node& operator=(Node&& other) noexcept;

    void addEdge(const ShrdPtr<Edge>& edge);
    const LinkedListSmart<ShrdPtr<Edge>>& getEdges() const;
};

class Edge {
private:
    int weight_;
    ShrdPtr<Node> fromNode_;
    ShrdPtr<Node> toNode_;

public:
    Edge(int weight, const ShrdPtr<Node>& fromNode, const ShrdPtr<Node>& toNode);

    int getWeight() const;
    ShrdPtr<Node> getFromNode() const;
    ShrdPtr<Node> getToNode() const;
};

class Graph {
private:
    DynamicArraySmart<ShrdPtr<Node>> nodes_;
    DynamicArraySmart<ShrdPtr<Edge>> edges_;

public:
    Graph() = default;

    ShrdPtr<Node> createNode();
    ShrdPtr<Node> getNodeByName(const std::string& name);
    ShrdPtr<Node> createNode(const std::string& nodeName);
    ShrdPtr<Edge> createEdge(int weight, const ShrdPtr<Node>& fromNode, const ShrdPtr<Node>& toNode);

    const DynamicArraySmart<ShrdPtr<Node>>& getNodes() const;
    const DynamicArraySmart<ShrdPtr<Edge>>& getEdges() const;
};

class Dijkstra {
public:
    static DynamicArraySmart<ShrdPtr<Edge>> findShortestPath(const ShrdPtr<Node>& startNode, const ShrdPtr<Node>& targetNode);
};

class BellmanFord {
public:
    static DynamicArraySmart<ShrdPtr<Edge>> findShortestPath(
        const ShrdPtr<Node>& startNode,
        const ShrdPtr<Node>& targetNode,
        const DynamicArraySmart<ShrdPtr<Edge>>& edges,
        const DynamicArraySmart<ShrdPtr<Node>>& nodes
    );
};
