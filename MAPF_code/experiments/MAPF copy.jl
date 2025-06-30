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
collect(vertices(MAPF_code.timed_graph(instance.graph, 3)))
@profview for _ in 1:5
    @info MAPF_code.prioritized_planning_v2(instance)
end
num_agents

path_prioritized = MAPF_code.prioritized_planning_v2(instance)
MAPF_code.visualization(file_instance, instance, path_prioritized)

edge_conflict = []
for s1 in 1:length(instance.starts)
    for s2 in (s1 + 1):length(instance.starts)
        if s1 != s2
            for i in 1:min(length(path_prioritized[s1]), length(path_prioritized[s2]))
                if src(path_prioritized[s1][i]) == dst(path_prioritized[s2][i]) ||
                    src(path_prioritized[s2][i]) == dst(path_prioritized[s1][i])
                    push!(edge_conflict, path_prioritized[s1][i])
                end
            end
        end
    end
end

edge_conflict

independent_paths = MAPF_code.independent_shortest_paths(instance)
max_len = maximum(length(path) for path in independent_paths) * 3

n = nv(instance.graph)

paths = Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}(undef, length(instance.starts))
heuristic = MAPF_code.euclidean_heuristic(instance.goals[1], instance.width)
paths[1] = a_star(
    instance.graph,
    instance.starts[1],
    instance.goals[1],
    weights(instance.graph),
    heuristic,
)

rem_list = [dst(e) + n * t for (t, e) in enumerate(paths[1])]

mutable_graph = MAPF_code.TimeExpandedGraph(instance.graph, max_len, rem_list)

collect(vertices(mutable_graph))

vlist = collect(vertices(mutable_graph))
@show minimum(vlist)
@show maximum(vlist)
@show length(vlist)
@show sort(vlist)[1:10]
@show sort(vlist)[(end - 9):end]

MAPF_code.training_LR([instance], 0.5, 10, 1e-5, 5000)