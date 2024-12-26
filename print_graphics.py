import matplotlib.pyplot as plt

# Считываем данные из файла
file_name = "test_data.txt"
times_dijkstra = []
times_bellman_ford = []
graph_sizes = []

with open(file_name, "r") as file:
    for line in file:
        dijkstra, bellman_ford, graph_size = map(int, line.split())
        times_dijkstra.append(dijkstra)
        times_bellman_ford.append(bellman_ford)
        graph_sizes.append(graph_size)

# Построение графика
plt.figure(figsize=(10, 6))

plt.plot(graph_sizes, times_dijkstra, label="Dijkstra", marker="o")
plt.plot(graph_sizes, times_bellman_ford, label="Ford-Bellman", marker="x")

plt.xlabel("Graph Size (Vertices x Edges)", fontsize=12)
plt.ylabel("Time (ms)", fontsize=12)
plt.title("Performance Comparison: Dijkstra vs Ford-Bellman", fontsize=14)
plt.legend()
plt.grid(True)

# Отображение графика
plt.show()