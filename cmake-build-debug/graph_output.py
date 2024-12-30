import networkx as nx
import matplotlib.pyplot as plt
G = nx.DiGraph()
G.add_node("C")
G.add_node("D")
G.add_node("E")
G.add_edge("C", "D", weight=10)
G.add_edge("D", "E", weight=2)
pos = nx.spring_layout(G)
weights = nx.get_edge_attributes(G, 'weight')
nx.draw(G, pos, with_labels=True, node_color='skyblue', edge_color='black', node_size=200, font_size=15)
nx.draw_networkx_edge_labels(G, pos, edge_labels=weights)
plt.show()
