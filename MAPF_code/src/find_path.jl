function independent_shortest_paths(instance::MAPF_Instance)
    paths = [
        a_star(instance.graph, instance.starts[s], instance.goals[s]) for
        s in 1:length(instance.starts)
    ]
    return paths
end

"""
	conflict_verification(s1, s2)

checks conflict locations between two agents s1 and s2

# Arguments
 - 's1', 's2': Agents being considered

# Returns
 - 'conflicts': List with graph vertices where there are conflicts between designed paths for s1 and s2
"""
function conflict_verification(s1, s2, paths)
    conflicts = []
    if s1 != s2
        for i in 1:min(length(paths[s1]), length(paths[s2]))
            if dst(paths[s1][i]) == dst(paths[s2][i]) || (
                dst(paths[s1][i]) == src(paths[s2][i]) &&
                dst(paths[s2][i]) == src(paths[s1][i])
            )
                push!(conflicts, paths[s1][i])
                push!(conflicts, paths[s2][i])
            end
        end
    end
    return conflicts
end

function prioritized_planning(instance::MAPF_Instance)
    paths = Vector{Vector{Edge}}(undef, length(instance.starts))
    for agent in size(instance.starts)
        if agent == 1
            paths[agent] = a_star(
                instance.graph, instance.starts[agent], instance.goals[agent]
            )
        else
            paths[agent] = a_star(
                instance.graph, instance.starts[agent], instance.goals[agent]
            )
            instance_graph_copy = instance.graph
            for s1 in 1:(agent - 1)
                conflicts = conflict_verification(s1, agent, paths)
                while conflicts
                    for edge in conflicts
                        rem_edge!(instance_graph_copy, edge)
                        conflicts = [x for x in conflicts if x != edge]
                    end
                    paths[agent] = a_star(
                        instance_graph_copy, instance.starts[agent], instance.goals[agent]
                    )
                    conflicts = conflict_verification(s1, agent, paths)
                end
            end
        end
    end
    return paths
end