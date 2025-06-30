function independent_shortest_paths(instance::MAPF_Instance)
    paths = [
        a_star(instance.graph, instance.starts[s], instance.goals[s]) for
        s in 1:length(instance.starts)
    ]
    return paths
end

function euclidean_heuristic(t::Int, width::Int)
    vertex_t = index_to_coords(t, width)

    return v -> begin
        vertex_v = index_to_coords(v, width)
        return sqrt((vertex_v[1] - vertex_t[1])^2 + (vertex_v[2] - vertex_t[2])^2)
    end
end

function euclidean_heuristic_time_expanded(goal::Int, width::Int, n_original::Int)
    goal_coords = index_to_coords(goal, width)

    return v -> begin
        original_v = ((v - 1) % n_original) + 1
        vertex_coords = index_to_coords(original_v, width)
        return sqrt(
            (vertex_coords[1] - goal_coords[1])^2 + (vertex_coords[2] - goal_coords[2])^2,
        )
    end
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

function timed_graph(g::SimpleWeightedGraph, max_len::Int)
    n = nv(g)
    sources = Vector{Int}()
    dsts = Vector{Int}()
    for v in 1:n
        for v1 in neighbors(g, v)
            for t in 1:max_len
                push!(sources, v + (t - 1) * n)
                push!(dsts, v1 + t * n)
            end
        end
    end
    weights = [1.0 for _ in 1:length(dsts)]
    timed_graph = SimpleWeightedGraph(sources, dsts, weights)

    return timed_graph
end

function unite_goal(instance::MAPF_Instance, teg::TimeExpandedGraph)
    n = nv(instance.graph)
    for t in 1:(teg.t - 1)
        src_idx = (t - 1) * n + goal
        dst_idx = t * n + goal
        if 1 <= src_idx <= nv(g) && 1 <= dst_idx <= nv(g)
            add_edge!(g, src_idx, dst_idx, 0.0)
        else
            println("Warning: Invalid edge indices in unite_goal: $src_idx, $dst_idx")
        end
    end
    return g
end

function prioritized_planning_v2(instance::MAPF_Instance)
    paths = Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}(
        undef, length(instance.starts)
    )
    heuristic = euclidean_heuristic(instance.goals[1], instance.width)
    paths[1] = a_star(
        instance.graph,
        instance.starts[1],
        instance.goals[1],
        weights(instance.graph),
        heuristic,
    )
    independent_paths = independent_shortest_paths(instance)
    max_len = maximum(length(path) for path in independent_paths) * 2

    n = nv(instance.graph)

    # NOVO: Reservas de vértices e arestas
    vertex_reservations = Dict{Tuple{Int,Int},Bool}()  # (v, t)
    edge_reservations = Dict{Tuple{Int,Int,Int},Bool}()  # (u, v, t)

    for (t, e) in enumerate(paths[1])
        u = src(e)
        v = dst(e)

        vertex_reservations[(v, t)] = true
        edge_reservations[(u, v, t)] = true
        edge_reservations[(v, u, t)] = true  # evitar troca
    end
    for pos in (length(paths[1]) + 1):max_len
        vertex_reservations[(instance.goals[1], pos)] = true
    end

    println("Creating mutable_graph")

    # Montar rem_v e rem_e com base nas reservas
    rem_vertices = [v + n * t for ((v, t), _) in vertex_reservations]
    rem_edges = SimpleWeightedEdge{Int,Float64}[]

    for ((u, v, t), _) in edge_reservations
        src_time_expanded = u + n * (t - 1)
        dst_time_expanded = v + n * t
        edge = SimpleWeightedEdge(
            src_time_expanded, dst_time_expanded, weights(instance.graph)[u, v]
        )
        push!(rem_edges, edge)
    end

    mutable_graph = TimeExpandedGraph(instance.graph, max_len, rem_vertices, rem_edges)

    for agent in 2:length(instance.starts)
        paths[agent] = Vector{SimpleWeightedEdge{Int64,Float64}}()
        println(agent)

        heuristic = euclidean_heuristic_time_expanded(
            instance.goals[agent], instance.width, n
        )
        mutable_graph.goal = instance.goals[agent]

        new_path = a_star(
            mutable_graph,
            instance.starts[agent],
            instance.goals[agent] + (mutable_graph.t - 1) * n,
            weights(mutable_graph),
            heuristic,
            SimpleWeightedEdge{Int,Float64},
        )

        while isempty(new_path)
            mutable_graph.t += 1

            new_path = a_star(
                mutable_graph,
                instance.starts[agent],
                instance.goals[agent] + (mutable_graph.t - 1) * n,
                weights(mutable_graph),
                heuristic,
                SimpleWeightedEdge{Int,Float64},
            )
        end

        W = weights(mutable_graph)
        n_sg = nv(instance.graph)
        new_edges = Vector{SimpleWeightedEdge{Int}}(undef, length(new_path))

        for (i, e) in enumerate(new_path)
            u = src(e)
            v = dst(e)
            src_local = ((u - 1) % n_sg) + 1
            dst_local = ((v - 1) % n_sg) + 1

            new_edges[i] = SimpleWeightedEdge(src_local, dst_local, W[u, v])
        end

        paths[agent] = new_edges

        # ATUALIZA RESERVAS com o novo caminho
        for (t, e) in enumerate(paths[agent])
            u = src(e)
            v = dst(e)
            vertex_reservations[(v, t)] = true
            edge_reservations[(u, v, t)] = true
            edge_reservations[(v, u, t)] = true
        end
        for pos in (length(paths[agent]) + 1):(mutable_graph.t)
            vertex_reservations[(instance.goals[agent], pos)] = true
        end

        # Atualizar rem_v e rem_e para o próximo agente
        mutable_graph.rem_v = [v + n * t for ((v, t), _) in vertex_reservations]
        mutable_graph.rem_e = SimpleWeightedEdge{Int,Float64}[]
        for ((u, v, t), _) in edge_reservations
            src_time_expanded = u + n * (t - 1)
            dst_time_expanded = v + n * t
            edge = SimpleWeightedEdge(
                src_time_expanded, dst_time_expanded, weights(instance.graph)[u, v]
            )
            push!(mutable_graph.rem_e, edge)
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