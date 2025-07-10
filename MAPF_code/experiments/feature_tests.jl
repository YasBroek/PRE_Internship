using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs
using MultiAgentPathFinding

file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 11

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, num_agents)

paths = MAPF_code.independent_shortest_paths(instance)
@profview for _ in 1:5
    MAPF_code.extract_features(instance)
end
conflicts = MAPF_code.conflict_identifier(instance, paths)
@info conflicts
features = zeros(ne(instance.graph))
edge_list = collect(edges(instance.graph))

for edge in 1:ne(instance.graph)
    features[edge] = MAPF_code.harmonic_centrality(instance, edge_list[edge])
end
@info features