"""
    independent_shortest_paths(instance::MAPF_Instance) 
        -> Vector{Vector{SimpleWeightedEdge{Int64, Float64}}}

Compute the **independent shortest paths** for all agents in a MAPF instance.  

Each agent’s path is planned individually using the A* search algorithm, **ignoring all conflicts** 
with other agents. This gives a baseline solution that is generally not collision-free, but can 
serve as an initial heuristic or ordering reference for prioritized planning.

# Arguments
- `instance::MAPF_Instance`:
    A MAPF problem instance containing:
    - `graph`: The environment graph (undirected, weighted).
    - `starts`: Vector of start vertices (one per agent).
    - `goals`: Vector of goal vertices (one per agent).

# Algorithm
- For each agent:
  1. Run `a_star` from its start to its goal on `instance.graph`.
  2. Use uniform zero heuristic (`v -> 0.0`) → effectively Dijkstra’s algorithm.
  3. Collect the resulting path as a sequence of `SimpleWeightedEdge`.

# Returns
- `paths::Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}`:
    A vector of paths, one per agent.  
    Each path is the **shortest path** (ignoring other agents).
"""
function independent_shortest_paths(instance::MAPF_Instance)
    paths = [
        a_star(
            instance.graph,
            instance.starts[s],
            instance.goals[s],
            weights(instance.graph),
            v -> 0.0,
            SimpleWeightedEdge{Int,Float64},
        ) for s in 1:length(instance.starts)
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

function conflict_verification_gdalle(s1, s2, paths)
    conflicts = []
    if s1 != s2
        for i in 1:min(length(paths[s1]), length(paths[s2]))
            if paths[s1][i + 1] == paths[s2][i + 1] ||
                (paths[s1][i + 1] == paths[s2][i] && paths[s2][i + 1] == paths[s1][i])
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

"""
    prioritized_planning_v2(instance::MAPF_Instance) -> Vector{Vector{SimpleWeightedEdge{Int64, Float64}}}

Run a prioritized planning algorithm for multi-agent pathfinding (MAPF) on a grid-like instance.  
This function computes collision-free paths for multiple agents by planning sequentially, giving 
higher priority to agents with fewer potential goal conflicts.

# Arguments
- `instance::MAPF_Instance`: 
    A MAPF problem instance containing:
    - `graph`: The environment graph (undirected, weighted).
    - `width`: Width of the grid (must be ≤ 128).
    - `starts`: Vector of start vertices (one per agent).
    - `goals`: Vector of goal vertices (one per agent).

# Returns
- `paths::Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}`:
    A vector of paths, where `paths[i]` is the path for agent `i`.

"""

function prioritized_planning_v2(instance::MAPF_Instance)
    if instance.width > 128
        error("Instance dimentions must be lower or equal to 128x128")
    end
    paths = Vector{Vector{SimpleWeightedEdge{Int64,Float64}}}(
        undef, length(instance.starts)
    )
    heuristic = euclidean_heuristic(instance.goals[1], instance.width)
    independent_paths = independent_shortest_paths(instance)
    max_len = maximum(length(path) for path in independent_paths) * 2

    goal_conflicts = [[0, agent] for agent in 1:length(instance.goals)]
    for agent in 1:length(instance.goals)
        for path in independent_paths
            for edge in path
                if dst(edge) == instance.goals[agent]
                    goal_conflicts[agent][1] += 1
                end
            end
        end
    end
    ag_order = [g[2] for g in sort!(goal_conflicts; by=x -> x[1], rev=false)]
    solved = []

    paths[ag_order[1]] = a_star(
        instance.graph,
        instance.starts[ag_order[1]],
        instance.goals[ag_order[1]],
        weights(instance.graph),
        heuristic,
    )

    n = nv(instance.graph)

    vertex_reservations = Dict{Tuple{Int,Int},Bool}()
    edge_reservations = Dict{Tuple{Int,Int,Int},Bool}()

    for (t, e) in enumerate(paths[ag_order[1]])
        u = src(e)
        v = dst(e)

        vertex_reservations[(v, t)] = true
        edge_reservations[(u, v, t)] = true
        edge_reservations[(v, u, t)] = true
    end
    for pos in (length(paths[ag_order[1]]) + 1):max_len
        vertex_reservations[(instance.goals[ag_order[1]], pos)] = true
    end

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
    time_expanded_weights = TimeExpandedWeights(build_sparse_weights(mutable_graph))
    push!(solved, ag_order[1])
    popfirst!(ag_order)
    len = length(ag_order)

    while !isempty(ag_order)
        println("$(length(ag_order)) out of $len")
        paths[ag_order[1]] = Vector{SimpleWeightedEdge{Int64,Float64}}()

        heuristic = euclidean_heuristic_time_expanded(
            instance.goals[ag_order[1]], instance.width, n
        )
        mutable_graph.goal = instance.goals[ag_order[1]]

        new_path = a_star(
            mutable_graph,
            instance.starts[ag_order[1]],
            instance.goals[ag_order[1]] + (mutable_graph.t - 1) * n,
            time_expanded_weights.W_sg,
            heuristic,
            SimpleWeightedEdge{Int,Float64},
        )

        i = 0

        while isempty(new_path) && i < 50
            i += 1
            mutable_graph.t += 1
            time_expanded_weights = TimeExpandedWeights(build_sparse_weights(mutable_graph))
            for s1 in 1:(ag_order[1] - 1)
                vertex_reservations[(instance.goals[s1], mutable_graph.t)] = true
            end
            mutable_graph.rem_v = [v + n * t for ((v, t), _) in vertex_reservations]

            new_path = a_star(
                mutable_graph,
                instance.starts[ag_order[1]],
                instance.goals[ag_order[1]] + (mutable_graph.t - 1) * n,
                time_expanded_weights.W_sg,
                heuristic,
                SimpleWeightedEdge{Int,Float64},
            )
        end

        if isempty(new_path)
            push!(ag_order, popfirst!(ag_order))
            continue
        end

        W = time_expanded_weights.W_sg
        n_sg = nv(instance.graph)
        new_edges = Vector{SimpleWeightedEdge{Int}}(undef, length(new_path))

        for (i, e) in enumerate(new_path)
            u = src(e)
            v = dst(e)
            src_local = ((u - 1) % n_sg) + 1
            dst_local = ((v - 1) % n_sg) + 1

            new_edges[i] = SimpleWeightedEdge(src_local, dst_local, W[u, v])
        end

        paths[ag_order[1]] = new_edges

        for (t, e) in enumerate(paths[ag_order[1]])
            u = src(e)
            v = dst(e)
            vertex_reservations[(v, t)] = true
            edge_reservations[(u, v, t)] = true
            edge_reservations[(v, u, t)] = true
        end
        for pos in (length(paths[ag_order[1]]) + 1):(mutable_graph.t)
            vertex_reservations[(instance.goals[ag_order[1]], pos)] = true
        end

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
        push!(solved, ag_order[1])
        popfirst!(ag_order)
    end

    return paths
end

"""
    path_cost(instance::MAPF_Instance, paths::Vector{Vector{SimpleWeightedEdge}}) -> Float64

Compute the **total cost** of a set of planned agent paths in a MAPF instance.  
The cost is defined as the sum of the edge weights traversed by all agents, excluding 
staying at the goal position.

# Arguments
- `instance::MAPF_Instance`: 
    The MAPF problem instance containing the environment graph and goals.
- `paths::Vector{Vector{SimpleWeightedEdge}}`: 
    A vector of paths as produced by `prioritized_planning_v2`.

# Returns
- `total_cost::Float64`: 
    The sum of all edge traversal costs across all agents.
"""
function path_cost(instance, paths)
    total_cost = 0
    for path in 1:length(paths)
        for edge in paths[path]
            if src(edge) != instance.goals[path]
                total_cost += instance.graph.weights[src(edge), dst(edge)]
            end
        end
    end
    return total_cost
end

function path_to_binary_matrix(instance::MAPF_Instance, paths)
    edge_list = collect(edges(instance.graph))
    binary_variables = zeros(length(instance.starts), ne(instance.graph))
    for agent in 1:length(instance.starts)
        for edge in 1:ne(instance.graph)
            if edge_list[edge] in paths[agent]
                binary_variables[agent, edge] += 1
            else
                binary_variables[agent, edge] += 0
            end
        end
    end
    return binary_variables
end

"""
    path_to_binary_vector(instance::MAPF_Instance, paths::Vector{Vector{SimpleWeightedEdge}}) 
        -> SparseVector{Int}

Convert a set of agent paths into a **binary edge-usage vector** representation.

This function maps each edge in the graph to an index in a sparse vector.  
For every edge traversed by an agent’s path, the corresponding entry in the vector is incremented 
(by 1 per traversal).  

Effectively, this produces a compact numerical encoding of how often each edge is used across all agents.

# Arguments
- `instance::MAPF_Instance`: 
    A MAPF problem instance containing the environment graph.
- `paths::Vector{Vector{SimpleWeightedEdge}}`: 
    A vector of agent paths (e.g. produced by `prioritized_planning_v2`).

# Returns
- `binary_variables::SparseVector{Int}`:  
    A sparse vector of length equal to the number of edges in the graph.  
    Entry `i` represents how many times edge `i` is used across all paths.
"""
function path_to_binary_vector(instance, paths)
    edge_list = collect(edges(instance.graph))
    edge_to_index = Dict((src(e), dst(e)) => i for (i, e) in enumerate(edge_list))
    binary_variables = spzeros(length(edge_list))
    for agent_path in paths
        for e in agent_path
            key = (min(src(e), dst(e)), max(src(e), dst(e)))
            i = get(edge_to_index, key, nothing)
            if i !== nothing
                binary_variables[i] += 1
            end
        end
    end
    return binary_variables
end

function path_to_binary_vector_gdalle(instance, solution)
    edge_list = collect(edges(instance.graph))
    binary_variables = spzeros(length(edge_list))
    for agent_path in solution.paths
        for v in 1:(length(agent_path) - 1)
            e = SimpleWeightedEdge(
                agent_path[v], agent_path[v + 1], instance.graph.weights[v, v + 1]
            )
            for (index, edge) in enumerate(edge_list)
                if e == edge
                    binary_variables[index] += 1
                end
            end
        end
    end
    return binary_variables
end