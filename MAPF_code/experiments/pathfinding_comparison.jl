using MultiAgentPathFinding
using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs

"Open map"
file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 10

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

path_gdalle = cooperative_astar(
    MAPF(instance.graph, instance.starts, instance.goals), collect(1:num_agents)
)

path_prioritized = MAPF_code.prioritized_planning_v2(instance)
MAPF_code.visualization(file_instance, instance, path_prioritized)

@profview for _ in 1:5
    MAPF_code.prioritized_planning_v2(instance)
end

paths_gdalle = [
    [
        SimpleWeightedEdge(
            path_gdalle.paths[k][i],
            path_gdalle.paths[k][i + 1],
            weights(instance.graph)[path_gdalle.paths[k][i], path_gdalle.paths[k][i + 1]],
        ) for i in 1:(length(path_gdalle.paths[k]) - 1)
    ] for k in 1:length(path_gdalle.paths)
]

MAPF_code.path_cost(instance, paths_gdalle)

MAPF_code.path_cost(instance, path_prioritized)