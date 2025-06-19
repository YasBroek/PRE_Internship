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
            if isempty(paths[agent])
                error("No path found for agent $agent")
            end
        else
            paths[agent] = a_star(
                instance.graph, instance.starts[agent], instance.goals[agent]
            )
            if isempty(paths[agent])
                error("No path found for agent $agent")
            end

            pos = instance.starts[agent]
            i = 1
            conflict_agent = 0
            while pos != instance.goals[agent]
                instance_copy = deepcopy(instance)
                conflict = false
                conflict_found = false
                new_path = a_star(instance.graph, pos, instance.goals[agent])

                for s1 in 1:(agent-1)
                    if i <= length(paths[s1]) &&
                       i <= length(new_path) &&
                       (
                           dst(new_path[i]) == dst(paths[s1][i]) ||
                           new_path[i] == paths[s1][i]
                       )
                        conflict = true
                        conflict_agent = s1
                        rem_edge!(instance_copy.graph, new_path[i])
                    end
                end

                while conflict
                    new_path = a_star(instance_copy.graph, pos, instance_copy.goals[agent])
                    if isempty(new_path)
                        if dst(paths[conflict_agent][i+1]) != pos &&
                           dst(paths[conflict_agent][i+1]) !=
                           dst(paths[conflict_agent][i])
                            paths[agent][i] = SimpleWeightedEdge(pos, pos)
                            conflict = false
                        else
                            list = deepcopy(paths[conflict_agent])
                            neighbor = rand([
                                x for x in
                                neighbors(instance.graph, dst(paths[conflict_agent][i])) if
                                x != pos &&
                                x != src(paths[conflict_agent][i]) &&
                                x != dst(paths[conflict_agent][i])
                            ])
                            paths[conflict_agent] = vcat(
                                list[1:i],
                                [
                                    SimpleWeightedEdge(dst(list[i]), neighbor),
                                    SimpleWeightedEdge(neighbor, dst(list[i])),
                                ],
                                list[(i+1):length(list)],
                            )
                            paths[agent][i] = SimpleWeightedEdge(pos, dst(list[i]))
                            conflict = false
                        end
                    else
                        for s1 in 1:(agent-1)
                            if i <= length(paths[s1]) &&
                               i <= length(new_path) &&
                               (
                                   dst(new_path[i]) == dst(paths[s1][i]) ||
                                   new_path[i] == paths[s1][i]
                               )
                                conflict_found = true
                                rem_edge!(instance_copy.graph, new_path[i])
                            end
                        end
                    end
                    if !conflict_found
                        conflict = false
                    end
                end
                pos = dst(paths[agent][i])
                i += 1
            end
        end
        println(agent)
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

function path_to_binary_matrix(instance::MAPF_Instance, paths)
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

function path_to_binary_vector(instance::MAPF_Instance, paths)
    edge_list = collect(edges(instance.graph))
    binary_variables = Vector{Int}(undef, ne(instance.graph))
    for edge in 1:ne(instance.graph)
        binary_variables[edge] = 0
        for agent in 1:length(instance.starts)
            if edge_list[edge] in paths[agent]
                binary_variables[edge] += 1
            end
        end
    end
    return binary_variables
end