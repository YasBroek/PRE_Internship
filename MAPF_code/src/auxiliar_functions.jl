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

function edge_betweenness_centrality(mapf, edge)
    graph = mapf.graph
    n_vertices = nv(graph)
    edge_betweenness = 0.0

    # For all pairs of vertices
    for s in 1:n_vertices
        for t in (s + 1):n_vertices
            if s != t
                # Find all shortest paths from s to t
                distances = dijkstra_shortest_paths(graph, s)

                if distances.dists[t] < Inf
                    # Count shortest paths that use this edge
                    paths_through_edge = count_paths_through_edge(
                        graph, s, t, edge, distances.dists[t]
                    )
                    total_shortest_paths = count_shortest_paths(
                        graph, s, t, distances.dists[t]
                    )

                    if total_shortest_paths > 0
                        edge_betweenness += paths_through_edge / total_shortest_paths
                    end
                end
            end
        end
    end

    # Normalize by the maximum possible betweenness
    max_pairs = (n_vertices * (n_vertices - 1)) / 2
    return max_pairs > 0 ? edge_betweenness / max_pairs : 0.0
end

"""
Helper function to count shortest paths from s to t that go through a specific edge.
"""
function count_paths_through_edge(graph, s, t, edge, target_distance)
    src_vertex = src(edge)
    dst_vertex = dst(edge)

    # Distance from s to edge source + edge weight + distance from edge destination to t
    dist_s_to_src = dijkstra_shortest_paths(graph, s).dists[src_vertex]
    dist_dst_to_t = dijkstra_shortest_paths(graph, dst_vertex).dists[t]

    path_length = dist_s_to_src + 1.0 + dist_dst_to_t  # Assuming edge weight = 1

    return abs(path_length - target_distance) < 1e-6 ? 1 : 0
end

"""
Helper function to count total number of shortest paths between two vertices.
"""
function count_shortest_paths(graph, s, t, target_distance)
    # For simplicity, return 1 if path exists (can be made more sophisticated)
    return target_distance < Inf ? 1 : 0
end

"""
Compute bridge criticality - how much removing this edge would impact connectivity.
Higher values indicate more critical bridges.
"""
function bridge_criticality(mapf, edge)
    graph = mapf.graph
    n_vertices = nv(graph)

    # Original connectivity measure
    original_connected_pairs = count_connected_pairs(graph)

    # Create a copy of the graph without this edge
    temp_graph = deepcopy(graph)
    rem_edge!(temp_graph, edge)

    # New connectivity measure
    new_connected_pairs = count_connected_pairs(temp_graph)

    # Return the relative impact (0 to 1, where 1 means removing edge disconnects everything)
    return if original_connected_pairs > 0
        (original_connected_pairs - new_connected_pairs) / original_connected_pairs
    else
        0.0
    end
end

"""
Helper function to count connected pairs in a graph.
"""
function count_connected_pairs(graph)
    n_vertices = nv(graph)
    connected_pairs = 0

    for s in 1:n_vertices
        distances = dijkstra_shortest_paths(graph, s)
        for t in (s + 1):n_vertices
            if distances.dists[t] < Inf
                connected_pairs += 1
            end
        end
    end

    return connected_pairs
end

"""
Compute path length impact - average increase in path length for all agents if this edge is removed.
"""
function path_length_impact_gdalle(mapf, edge, original_paths)
    total_impact = 0.0
    n_agents = length(mapf.agents)

    # Create graph without this edge
    temp_graph = deepcopy(mapf.graph)
    rem_edge!(temp_graph, edge)
    temp_mapf = MAPF(mapf.agents, temp_graph)

    for agent_idx in 1:n_agents
        agent = mapf.agents[agent_idx]

        # Original path length
        original_length = length(original_paths[agent_idx]) - 1  # -1 because path includes start

        # New path length without this edge
        try
            new_path = dijkstra_shortest_path(temp_graph, agent.start, agent.goal)
            new_length = length(new_path) - 1

            # Impact is the relative increase
            if original_length > 0
                impact = (new_length - original_length) / original_length
                total_impact += max(0, impact)  # Only count increases
            end
        catch
            # If no path exists, this edge is critical for this agent
            total_impact += 1.0  # Maximum impact
        end
    end

    return n_agents > 0 ? total_impact / n_agents : 0.0
end

"""
Compute path diversity score - how many different agent paths could potentially use this edge.
Considers both current paths and reasonable alternatives.
"""
function path_diversity_score_gdalle(mapf, edge, original_paths)
    n_agents = length(mapf.agents)
    agents_could_use = 0

    src_vertex = src(edge)
    dst_vertex = dst(edge)

    for agent_idx in 1:n_agents
        agent = mapf.agents[agent_idx]

        # Check if this edge could be on a reasonable path for this agent
        # Distance from agent start to edge source
        dist_start_to_edge = dijkstra_shortest_paths(mapf.graph, agent.start).dists[src_vertex]
        # Distance from edge destination to agent goal  
        dist_edge_to_goal = dijkstra_shortest_paths(mapf.graph, dst_vertex).dists[agent.goal]

        # Direct distance from start to goal
        direct_distance = dijkstra_shortest_paths(mapf.graph, agent.start).dists[agent.goal]

        # Path through edge distance
        path_through_edge = dist_start_to_edge + 1.0 + dist_edge_to_goal  # +1 for edge itself

        # Consider it a viable path if it's not too much longer than direct path
        detour_ratio = direct_distance > 0 ? path_through_edge / direct_distance : Inf

        # Also check if agent currently uses this edge
        currently_uses = edge_in_path(original_paths[agent_idx], edge)

        # Count as potential if reasonable detour (< 50% longer) or currently uses
        if detour_ratio <= 1.5 || currently_uses
            agents_could_use += 1
        end
    end

    # Return as fraction of total agents
    return n_agents > 0 ? agents_could_use / n_agents : 0.0
end

"""
Helper function to check if an edge is used in a path.
"""
function edge_in_path(path, target_edge)
    if length(path) < 2
        return false
    end

    for i in 1:(length(path) - 1)
        if (path[i] == src(target_edge) && path[i + 1] == dst(target_edge)) ||
            (path[i] == dst(target_edge) && path[i + 1] == src(target_edge))
            return true
        end
    end
    return false
end