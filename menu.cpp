#include "menu.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include <cstdlib>

void displayGraph(const Graph& graph) {
    std::ofstream outFile("graph_output.py");
    outFile << "import networkx as nx\n";
    outFile << "import matplotlib.pyplot as plt\n";
    outFile << "G = nx.DiGraph()\n";
    for (int i = 0; i < graph.getNodes().GetLength(); ++i) {
        auto& node = graph.getNodes()[i];
        outFile << "G.add_node(\"" << node->getName() << "\")\n";
    }
    for (int i = 0; i < graph.getEdges().GetLength(); ++i) {
        auto& edge = graph.getEdges()[i];
        outFile << "G.add_edge(\"" << edge->getFromNode()->getName() << "\", \"" << edge->getToNode()->getName() << "\", weight=" << edge->getWeight() << ")\n";
    }

    outFile << "pos = nx.spring_layout(G)\n";
    outFile << "weights = nx.get_edge_attributes(G, 'weight')\n";
    outFile << "nx.draw(G, pos, with_labels=True, node_color='skyblue', edge_color='black', node_size=200, font_size=15)\n";
    outFile << "nx.draw_networkx_edge_labels(G, pos, edge_labels=weights)\n";
    outFile << "plt.show()\n";

    outFile.close();
    system("python3 graph_output.py");
}

void menu() {
    Graph graph;

    auto nodeA = graph.createNode("A");
    auto nodeB = graph.createNode("B");
    auto nodeC = graph.createNode("C");
    auto nodeD = graph.createNode("D");
    auto nodeE = graph.createNode("E");

    graph.createEdge(10, nodeA, nodeB);
    graph.createEdge(15, nodeA, nodeC);
    graph.createEdge(12, nodeB, nodeD);
    graph.createEdge(10, nodeC, nodeD);
    graph.createEdge(2, nodeD, nodeE);
    graph.createEdge(1, nodeC, nodeE);

    while (true) {
        std::cout << "\n=== Graph Menu ===\n";
        std::cout << "1. Display Graph\n";
        std::cout << "2. Find Shortest Path (Dijkstra)\n";
        std::cout << "3. Find Shortest Path (Bellman-Ford)\n";
        std::cout << "4. Add Node\n";
        std::cout << "5. Add Edge\n";
        std::cout << "6. Exit\n";
        std::cout << "Choose an option: ";

        int choice;
        std::cin >> choice;

        if (choice == 1) {
            displayGraph(graph);

        } else if (choice == 2) {
            bool isContinuable = true;
            for (int i = 0; i < graph.getEdges().GetLength(); ++i) {
                if (graph.getEdges()[i]->getWeight() < 0) {
                    std::cout << "You can not use Dijkstra algorythm with negative edges weight";
                    isContinuable = false;
                }
            }
            if (!isContinuable) {
                continue;
            }
            std::string startNode, targetNode;
            std::cout << "Enter start node: ";
            std::cin >> startNode;
            std::cout << "Enter target node: ";
            std::cin >> targetNode;
            ShrdPtr<Node> start;
            ShrdPtr<Node> target;
            try {
                start = graph.getNodeByName(startNode);
                target = graph.getNodeByName(targetNode);
            } catch (const std::exception& e) {
                std::cout << e.what() << "\n";
                continue;
            }

            if (start && target) {
                auto shortestPath = Dijkstra::findShortestPath(start, target);

                std::cout << "Shortest path from " << startNode << " to " << targetNode << ":\n";
                int totalWeight = 0;
                for (const auto& edge : shortestPath) {
                    std::cout << edge->getFromNode()->getName() << " -> "
                              << edge->getToNode()->getName() << " (Weight: "
                              << edge->getWeight() << ")\n";
                    totalWeight += edge->getWeight();
                }

                std::cout << "Total path weight: " << totalWeight << "\n";
            } else {
                std::cout << "Invalid node names.\n";
            }

        } else if (choice == 3) {
            std::string startNode, targetNode;
            std::cout << "Enter start node: ";
            std::cin >> startNode;
            std::cout << "Enter target node: ";
            std::cin >> targetNode;

            ShrdPtr<Node> start;
            ShrdPtr<Node> target;
            try {
                start = graph.getNodeByName(startNode);
                target = graph.getNodeByName(targetNode);
            } catch (const std::exception& e) {
                std::cout << e.what() << "\n";
                continue;
            }
            try {
                auto shortestPath = BellmanFord::findShortestPath(start, target, graph.getEdges(), graph.getNodes());

                std::cout << "Shortest path from " << startNode << " to " << targetNode << ":\n";
                int totalWeight = 0;
                for (const auto& edge : shortestPath) {
                    std::cout << edge->getFromNode()->getName() << " -> "
                              << edge->getToNode()->getName() << " (Weight: "
                              << edge->getWeight() << ")\n";
                    totalWeight += edge->getWeight();
                }

                std::cout << "Total path weight: " << totalWeight << "\n";
            } catch (const std::exception& e) {
                std::cout << e.what() << "\n";
            }

        } else if (choice == 4) {
            std::string nodeName;
            std::cout << "Enter node name: ";
            std::cin >> nodeName;
            graph.createNode(nodeName);
            std::cout << "Node " << nodeName << " added.\n";

        } else if (choice == 5) {
            try {
                std::string fromNodeName, toNodeName;
                int weight;
                std::cout << "Enter start node: ";
                std::cin >> fromNodeName;
                std::cout << "Enter target node: ";
                std::cin >> toNodeName;
                std::cout << "Enter edge weight: ";
                std::cin >> weight;

                ShrdPtr<Node> fromNode = graph.getNodeByName(fromNodeName);
                ShrdPtr<Node> toNode = graph.getNodeByName(toNodeName);

                if (fromNode && toNode) {
                    graph.createEdge(weight, fromNode, toNode);
                    std::cout << "Edge from " << fromNodeName << " to " << toNodeName << " with weight " << weight << " added.\n";
                } else {
                    std::cout << "Invalid node names.\n";
                }
            } catch (const std::exception& e) {
                std::cout << e.what() << "\n";
            }

        } else if (choice == 6) {
            break;

        } else {
            std::cout << "Invalid choice. Try again.\n";
        }
    }
}
