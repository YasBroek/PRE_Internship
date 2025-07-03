using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs
using MultiAgentPathFinding

"Open map"
file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 11

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

mapf = MAPF(instance.graph, instance.starts, instance.goals)

paths = MAPF_code.independent_shortest_paths(instance)
pathsG = independent_dijkstra(mapf)
@info MAPF_code.Solution_to_paths(pathsG, instance)

MAPF_code.path_cost(instance, paths)
sum_of_costs(pathsG, mapf)

PP = MAPF_code.prioritized_planning_v2(instance)

list = [[s] for s in instance.starts]

for path in 1:length(PP)
    for edge in PP[path]
        push!(list[path], dst(edge))
    end
end

list
PP

is_feasible(Solution(list), mapf)
