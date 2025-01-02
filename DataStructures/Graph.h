#pragma once

#include "LinkedListSmart.h"
#include "ShrdPtr.h"
#include "HashTable.h"

#include <functional>
#include <queue>
#include <utility>
#include <limits>
#include <stdexcept>
#include <string>

template <typename T>
class Edge;

template <typename T>
class Node;

template <typename T>
class Graph;

template <typename T>
class Node {
private:
    LinkedListSmart<ShrdPtr<Edge<T>>> edges_;
    T name_;
    static int nodeId_;

public:
    Node() { name_ = std::to_string(++nodeId_); }
    explicit Node(T name) : name_(std::move(name)) {}

    const T& getName() const { return name_; }

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    Node(Node&& other) noexcept = default;
    Node& operator=(Node&& other) noexcept = default;

    void addEdge(const ShrdPtr<Edge<T>>& edge) { edges_.Append(edge); }
    LinkedListSmart<ShrdPtr<Edge<T>>>& getEdges() { return edges_; }
};

template <typename T>
int Node<T>::nodeId_ = 0;

template <typename T>
class Edge {
private:
    int weight_;
    ShrdPtr<Node<T>> fromNode_;
    ShrdPtr<Node<T>> toNode_;

public:
    Edge(int weight, const ShrdPtr<Node<T>>& fromNode, const ShrdPtr<Node<T>>& toNode)
        : weight_(weight), fromNode_(fromNode), toNode_(toNode) {}

    int getWeight() const { return weight_; }
    ShrdPtr<Node<T>> getFromNode() const { return fromNode_; }
    ShrdPtr<Node<T>> getToNode() const { return toNode_; }
};

template <typename T>
class Graph {
private:
    DynamicArraySmart<ShrdPtr<Node<T>>> nodes_;
    DynamicArraySmart<ShrdPtr<Edge<T>>> edges_;

public:

    ShrdPtr<Node<T>> createNode(const T& nodeName) {
        auto node = ShrdPtr<Node<T>>(new Node<T>(nodeName));
        nodes_.Append(node);
        return node;
    }

    ShrdPtr<Node<T>> getNodeByName(const T& name) {
        for (const auto& node : nodes_) {
            if (node->getName() == name) {
                return node;
            }
        }
        throw std::runtime_error("Node not found");
    }

    ShrdPtr<Edge<T>> createEdge(int weight, const ShrdPtr<Node<T>>& fromNode, const ShrdPtr<Node<T>>& toNode) {
        auto edge = ShrdPtr<Edge<T>>(new Edge<T>(weight, fromNode, toNode));
        edges_.Append(edge);
        fromNode->addEdge(edge);
        return edge;
    }

    void removeNode(const T& nodeName);
    void removeEdge(const T& fromNodeName, const T& toNodeName);

    const DynamicArraySmart<ShrdPtr<Node<T>>>& getNodes() const { return nodes_; }
    const DynamicArraySmart<ShrdPtr<Edge<T>>>& getEdges() const { return edges_; }
};

class Dijkstra {
public:
    template <typename T>
    static DynamicArraySmart<ShrdPtr<Edge<T>>> findShortestPath(const ShrdPtr<Node<T>>& startNode, const ShrdPtr<Node<T>>& targetNode);
};

class BellmanFord {
public:
    template <typename T>
    static DynamicArraySmart<ShrdPtr<Edge<T>>> findShortestPath(
        const ShrdPtr<Node<T>>& startNode,
        const ShrdPtr<Node<T>>& targetNode,
        const DynamicArraySmart<ShrdPtr<Edge<T>>>& edges,
        const DynamicArraySmart<ShrdPtr<Node<T>>>& nodes
    );
};

template <typename T>
void Graph<T>::removeNode(const T& nodeName) {
    ShrdPtr<Node<T>> nodeToRemove;
    for (int i = 0; i < nodes_.GetLength(); ++i) {
        if (nodes_[i]->getName() == nodeName) {
            nodeToRemove = nodes_[i];
            nodes_.RemoveAt(i);
            break;
        }
    }

    if (!nodeToRemove) {
        throw std::invalid_argument("Node does not exist");
    }

    for (int i = edges_.GetLength() - 1; i >= 0; --i) {
        if (edges_[i]->getFromNode() == nodeToRemove || edges_[i]->getToNode() == nodeToRemove) {
            edges_.RemoveAt(i);
        }
    }
}

template <typename T>
void Graph<T>::removeEdge(const T& fromNode, const T& toNode) {
    ShrdPtr<Edge<T>> edgeToRemove;
    for (int i = 0; i < edges_.GetLength(); ++i) {
        if (edges_[i]->getFromNode()->getName() == fromNode && edges_[i]->getToNode()->getName() == toNode) {
            edgeToRemove = edges_[i];
            edges_.RemoveAt(i);
            break;
        }
    }

    if (!edgeToRemove) {
        throw std::invalid_argument("Edge does not exist");
    }

    for (auto& edge : edgeToRemove->getFromNode()->getEdges()) {
        if (edge == edgeToRemove) {
            edgeToRemove->getFromNode()->getEdges().Remove(edge);
            break;
        }
    }

    for (auto& edge : edgeToRemove->getToNode()->getEdges()) {
        if (edge == edgeToRemove) {
            edgeToRemove->getToNode()->getEdges().Remove(edge);
            break;
        }
    }
}
