using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs

"Open map"
file_instance = readlines(
    open("MAPF_code/input/maze-128-128-10/instance/maze-128-128-10.map")
)

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/maze-128-128-10/instance/maze-128-128-10-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 7

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)
collect(vertices(MAPF_code.timed_graph(instance.graph, 3)))
@profview for _ in 1:5
    @info MAPF_code.timed_graph(instance.graph, 3)
end
num_agents

path_prioritized = MAPF_code.prioritized_planning_v2(instance)

MAPF_code.visualization(file_instance, instance, path_prioritized)

@info MAPF_code.prioritized_planning(instance)

@info MAPF_code.independent_shortest_paths(instance)

using Graphs
println("Has path: ", has_path(instance.graph, instance.starts[2], instance.goals[2]))

@info neighbors(instance.graph, instance.starts[7])

MAPF_code.training_LR(instance, 0.5, 10, 0.001, 500)

@info MAPF_code.path_to_binary(instance, MAPF_code.independent_shortest_paths(instance))

@info instance.goals[10]

cc = 0
lista = []
for agent in 1:length(instance.starts)
    for s1 in 1:length(instance.starts)
        if s1 != agent
            for i in 1:min(length(path_prioritized[s1]), length(path_prioritized[agent]))
                if dst(path_prioritized[agent][i]) == dst(path_prioritized[s1][i]) ||
                    path_prioritized[agent][i] == path_prioritized[s1][i] ||
                    (
                        dst(path_prioritized[agent][i]) == src(path_prioritized[s1][i]) &&
                        (src(path_prioritized[agent][i]) == dst(path_prioritized[s1][i]))
                    )
                    cc += 1
                    push!(
                        lista,
                        (agent, s1, path_prioritized[agent][i], path_prioritized[s1][i]),
                    )
                end
            end
        end
    end
end

cc
@info lista

for agent in 1:length(instance.starts)
    for s1 in 1:length(instance.starts)
        if s1 != agent
            for i in 1:min(length(path_prioritized[s1]), length(path_prioritized[agent]))
                e1 = path_prioritized[agent][i]
                e2 = path_prioritized[s1][i]
                if dst(e1) == dst(e2) ||
                    e1 == e2 ||
                    (dst(e1) == src(e2) && src(e1) == dst(e2))
                    println("Conflito t=$i entre Agente $agent e $s1:")
                    println("    $agent: ", src(e1), " -> ", dst(e1))
                    println("    $s1: ", src(e2), " -> ", dst(e2))
                end
            end
        end
    end
end

g = SimpleWeightedGraph(3)

add_edge!(g, 1, 2, 0.5);

add_edge!(g, 2, 3, 0.8);

add_edge!(g, 1, 3, 2.0);