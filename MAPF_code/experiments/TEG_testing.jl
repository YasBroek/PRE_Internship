using Revise
using MAPF_code
using Graphs

normal_graph = MAPF_code.SimpleWeightedGraph(4)
vertices(normal_graph)
edge_list = [(1, 2), (1, 1), (1, 4), (2, 3), (3, 3), (3, 4)]
for edge in edge_list
    add_edge!(normal_graph, edge[1], edge[2], 1.0)
end

edges(normal_graph)
neighbors(normal_graph, 2)

rem_v = [(3, 1), (2, 2), (1, 3)]
res = [i[1] + nv(normal_graph) * (i[2] - 1) for i in rem_v]

TEG = MAPF_code.TimeExpandedGraph(normal_graph, 3, res)

@info MAPF_code.edges(TEG)

@info MAPF_code.edgetype(TEG)

@info MAPF_code.has_edge(TEG, MAPF_code.SimpleWeightedEdge(1, 6))

@info MAPF_code.has_vertex(TEG, 16)

@info MAPF_code.outneighbors(TEG, 6)

@info MAPF_code.nv(TEG)
