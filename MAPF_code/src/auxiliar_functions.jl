function conflict_counter(conflicts, edge)
    count = 0
    for item in conflicts
        if src(edge) == src(item) && dst(edge) == dst(item)
            count += 1
        elseif src(edge) == dst(item) && dst(edge) == src(item)
            count += 1
        end
    end
    return count
end

function conflict_identifier(instance::MAPF_Instance, Solution)
    conflicts = []
    for s1 in 1:length(instance.starts)
        for s2 in (s1 + 1):length(instance.starts)
            for edge in conflict_verification_gdalle(s1, s2, Solution.paths)
                push!(conflicts, edge)
            end
        end
    end
    return conflicts
end

function conflict_identifier_gdalle(mapf, paths)
    conflicts = []
    for s1 in 1:length(mapf.departures)
        for s2 in (s1 + 1):length(mapf.departures)
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

function harmonic_centrality(instance, edge)
    ds = dijkstra_shortest_paths(instance.graph, dst(edge))

    harmonic_sum = sum(d == Inf ? 0.0 : 1.0 / d for d in ds.dists if d > 0)

    return harmonic_sum
end

function normalized_closeness_centrality(instance::MAPF_Instance, edge)
    ds = dijkstra_shortest_paths(instance.graph, dst(edge))
    finite_dists = filter(d -> d != Inf && d > 0, ds.dists)

    if isempty(finite_dists)
        return 0.0
    end

    n_reachable = length(finite_dists)
    sum_distances = sum(finite_dists)

    return (n_reachable - 1)
end

function index_to_coords_gdalle(v)
    grid = read_benchmark_map("room-32-32-4")
    tuple = parse_benchmark_map(grid)
    return tuple[end][v]
end

function distance_to_closest_obstacle_gdalle(mapf, edge, tuple)
    obstacles = [v for v in collect(vertices(mapf.graph)) if degree(mapf.graph, v) == 0]
    min_dist = Inf
    for o in obstacles
        vertex_v = tuple[end][dst(edge)]
        vertex_t = tuple[end][o]
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < min_dist
            min_dist = dist
        end
    end
    vertex_v = tuple[end][dst(edge)]
    dist_border_list = [
        vertex_v[1], vertex_v[2], 32 - vertex_v[1] + 1, 32 - vertex_v[2] + 1
    ]
    if min_dist > minimum(dist_border_list)
        min_dist = minimum(dist_border_list)
    end
    return min_dist
end

function distance_to_closest_obstacle(instance, edge)
    obstacles = [
        v for v in collect(vertices(instance.graph)) if degree(instance.graph, v) == 0
    ]
    min_dist = Inf
    for o in obstacles
        vertex_v = index_to_coords(dst(edge), instance.width)
        vertex_t = index_to_coords(o, instance.width)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < min_dist
            min_dist = dist
        end
    end
    vertex_v = index_to_coords(dst(edge), instance.width)
    dist_border_list = [
        vertex_v[1],
        vertex_v[2],
        instance.width - vertex_v[1] + 1,
        instance.height - vertex_v[2] + 1,
    ]
    if min_dist > minimum(dist_border_list)
        min_dist = minimum(dist_border_list)
    end
    return min_dist
end

function distance_to_closest_agent(instance::MAPF_Instance, edge)
    min_dist = Inf
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge), instance.width)
        vertex_t = index_to_coords(g, instance.width)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < min_dist
            min_dist = dist
        end
    end
    return min_dist
end

function distance_to_closest_agent_gdalle(mapf, edge, tuple)
    min_dist = Inf
    for g in mapf.departures
        vertex_v = tuple[end][dst(edge)]
        vertex_t = tuple[end][g]
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < min_dist
            min_dist = dist
        end
    end
    return min_dist
end

function distance_to_all_agents(instance::MAPF_Instance, edge)
    sum_dists = 0
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge), instance.width)
        vertex_t = index_to_coords(g, instance.width)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        sum_dists += dist
    end
    return sum_dists
end

function distance_to_all_agents_gdalle(mapf, edge, tuple)
    sum_dists = 0
    for g in mapf.departures
        vertex_v = tuple[end][dst(edge)]
        vertex_t = tuple[end][g]
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        sum_dists += dist
    end
    return sum_dists
end

function number_of_agents_close(instance::MAPF_Instance, edge)
    c = 0
    for g in instance.goals
        vertex_v = index_to_coords(dst(edge), instance.width)
        vertex_t = index_to_coords(g, instance.width)
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < sqrt(instance.width)
            c += 1
        end
    end
    return c
end

function number_of_agents_close_gdalle(mapf, edge, tuple)
    c = 0
    for g in mapf.departures
        vertex_v = tuple[end][dst(edge)]
        vertex_t = tuple[end][g]
        dist = sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
        if dist < sqrt(32)
            c += 1
        end
    end
    return c
end

function Solution_to_paths(s::Solution, instance)
    list = [[] for _ in 1:length(s.paths)]
    for i in 1:length(s.paths)
        for v in 1:(length(s.paths[i]) - 1)
            push!(
                list[i],
                SimpleWeightedEdge(
                    s.paths[i][v],
                    s.paths[i][v + 1],
                    weights(instance.graph)[s.paths[i][v], s.paths[i][v + 1]],
                ),
            )
        end
    end
    return list
end

function calculate_path(instance, regression_weights)
    features = extract_features(instance)
    weighted_instance = deepcopy(instance)
    θ = linear_regression(features, regression_weights)

    weighted_instance = adapt_weights(weighted_instance, θ)

    paths_vertices = cooperative_astar(
        MAPF(weighted_instance.graph, instance.starts, instance.goals),
        collect(1:length(instance.starts)),
    )
    paths = [
        [
            SimpleWeightedEdge(
                paths_vertices.paths[k][i],
                paths_vertices.paths[k][i + 1],
                weights(instance.graph)[
                    paths_vertices.paths[k][i], paths_vertices.paths[k][i + 1]
                ],
            ) for i in 1:(length(paths_vertices.paths[k]) - 1)
        ] for k in 1:length(paths_vertices.paths)
    ]

    return paths
end

function calculate_path_v(instance, regression_weights)
    features = extract_features(instance)
    weighted_instance = deepcopy(instance)
    θ = linear_regression(features, regression_weights)

    weighted_instance = adapt_weights(weighted_instance, θ)

    paths_vertices = cooperative_astar(
        MAPF(weighted_instance.graph, instance.starts, instance.goals),
        collect(1:length(instance.starts)),
    )
    return paths_vertices
end