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
    paths = Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}(
        undef, length(instance.starts)
    )
    for agent in 1:length(instance.starts)
        if agent == 1
            paths[agent] = a_star(
                instance.graph, instance.starts[agent], instance.goals[agent]
            )
        else
            paths[agent] = a_star(
                instance.graph, instance.starts[agent], instance.goals[agent]
            )
            if isempty(paths[agent])
                error("No path found for agent $agent")
            end

            pos = instance.starts[agent]
            i = 1
            while pos != instance.goals[agent]
                conflict = false
                instance_copy = deepcopy(instance)
                new_path = paths[agent]

                for s1 in 1:(agent - 1)
                    if i <= length(paths[s1]) &&
                        i <= length(new_path) &&
                        (
                            dst(new_path[i]) == dst(paths[s1][i]) ||  # Vertex conflict
                            new_path[i] == paths[s1][i]
                        )              # Edge conflict
                        conflict = true
                        # Remove conflicting edge from the copied graph
                        rem_edge!(instance_copy.graph, new_path[i])
                        break
                    end
                end

                while conflict
                    new_path = a_star(
                        instance_copy.graph,
                        instance_copy.starts[agent],
                        instance_copy.goals[agent],
                    )
                    if isempty(new_path)
                        push!(paths[agent], SimpleWeightedEdge(pos, pos))
                        conflict = false
                    end
                    conflict_found = false
                    for s1 in 1:(agent - 1)
                        if i <= length(paths[s1]) &&
                            i <= length(new_path) &&
                            (
                                dst(new_path[i]) == dst(paths[s1][i]) ||  # Vertex conflict
                                new_path[i] == paths[s1][i]
                            )              # Edge conflict
                            # Remove conflicting edge from the copied graph
                            rem_edge!(instance_copy.graph, new_path[i])
                            break
                        end
                    end
                    if !conflict_found
                        pos = dst(paths[agent][i])
                        conflict = false
                    end
                end
            end
        end
    end
    return paths
end

function path_cost(paths)
    total_cost = 0
    for path in paths
        total_cost = total_cost + length(path)
    end
    return total_cost
end

function path_to_binary(instance::MAPF_Instance, paths)
    edge_list = collect(edges(instance.graph))
    binary_variables = Matrix{Int}(undef, length(instance.starts), ne(instance.graph))
    for agent in 1:length(instance.starts)
        for edge in 1:ne(instance.graph)
            if edge_list[edge] in paths[agent]
                binary_variables[agent, edge] = 1
            else
                binary_variables[agent, edge] = 0
            end
        end
    end
    return binary_variables
end