function conflict_counter(conflicts, edge)
    c = 0
    for item in conflicts
        if edge == item
            c += 1
        end
    end
    return c
end

function conflict_identifier(instance::MAPF_Instance, paths)
    conflicts = []
    for s1 in 1:length(instance.starts)
        for s2 in (s1 + 1):length(instance.starts)
            for edge in conflict_verification(s1, s2, paths)
                push!(conflicts, edge)
            end
        end
    end
    return conflicts
end

function step_counter(paths, edge)
    step_counter = 0
    for path in paths
        for e in path
            if e == edge
                step_counter += 1
            end
        end
    end
    return step_counter
end

function closeness_centrality(instance::MAPF_Instance, edge)
    ds = dijkstra_shortest_paths(instance.graph, dst(edge))
    return mean(ds)
end

function distance_to_closest_obstacle(instance::MAPF_Instance, edge)
    obstacles = [v for v in collect(vertices(instance.graph)) if degree(v) == 0]
    max_dist = 0
    for o in obstacles
        vertex_v = index_to_coords(dst(edge))
        vertex_t = index_to_coords(o)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist > max_dist
            max_dist = dist
        end
    end
    return dist
end

function distance_to_closest_agent(instance::MAPF_Instance, edge)
    max_dist = 0
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge))
        vertex_t = index_to_coords(g)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist > max_dist
            max_dist = dist
        end
    end
    return dist
end

function distance_to_all_agents(instance::MAPF_Instance, edge)
    sum_dists = 0
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge))
        vertex_t = index_to_coords(g)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        sum_dists += dist
    end
    return sum_dists
end

function number_of_agents_close(instance::MAPF_Instance, edge)
    c = 0
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge))
        vertex_t = index_to_coords(g)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < sqrt(instance.width)
            c += 1
        end
    end
    return c
end