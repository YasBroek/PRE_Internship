Graphs.edgetype(::TimeExpandedGraph) = SimpleWeightedEdge{Int}
Graphs.has_edge(g::TimeExpandedGraph, e::SimpleWeightedEdge) = e in edges(g)
Graphs.has_vertex(g::TimeExpandedGraph, v::Int) = v in vertices(g)
Graphs.nv(g::TimeExpandedGraph) = Graphs.nv(g.s_g) * g.t
Graphs.is_directed(g::TimeExpandedGraph) = true
Graphs.is_directed(::Type{<:TimeExpandedGraph}) = true
Graphs.neighbors(g::TimeExpandedGraph, v::Int) = outneighbors(g, v)

function build_sparse_weights(g::TimeExpandedGraph)
    n = nv(g)
    s_n = nv(g.s_g)
    W_sg = Graphs.weights(g.s_g)

    I = Int[]
    J = Int[]
    V = Float64[]

    for e in edges(g)
        u, v = src(e), dst(e)
        orig_u = (u - 1) % s_n + 1
        orig_v = (v - 1) % s_n + 1
        if u < 1 || v < 1
            @warn "Índice inválido em build_sparse_weights: u=$u, v=$v"
        end

        push!(I, u)
        push!(J, v)
        push!(V, W_sg[orig_u, orig_v])
    end

    return sparse(I, J, V, n, n)
end

function Graphs.edges(g::TimeExpandedGraph)
    edge_list = SimpleWeightedEdge{Int,Float64}[]
    nv_sg = nv(g.s_g)

    for v in Graphs.vertices(g.s_g)
        for n in Graphs.neighbors(g.s_g, v)
            for i in 1:(g.t - 1)
                src_vertex = nv_sg * (i - 1) + v
                dst_vertex = nv_sg * i + n

                if dst_vertex in g.rem_v
                    continue
                end

                edge = SimpleWeightedEdge(src_vertex, dst_vertex, weights(g.s_g)[v, n])

                if edge in g.rem_e
                    continue
                end

                push!(edge_list, edge)
            end
        end
    end
    for i in 1:(g.t - 1)
        if g.goal > 0
            push!(
                edge_list,
                SimpleWeightedEdge(
                    g.goal + (i - 1) * nv(g.s_g), g.goal + i * nv(g.s_g), 0.0
                ),
            )
        end
    end
    return edge_list
end

function Graphs.vertices(g::TimeExpandedGraph)
    vertex_list = [
        Graphs.nv(g.s_g) * (i - 1) + v for v in 1:Graphs.nv(g.s_g) for i in 1:(g.t)
    ]
    """
    for item in g.rem
        filter!(x -> x != item, vertex_list)
    end
    """
    return vertex_list
end

function Graphs.inneighbors(g::TimeExpandedGraph, v::Int)
    if v in g.rem_v
        inneighbors = []
    elseif (v//Graphs.nv(g.s_g)) > 1
        inneighbors = [
            n + ((v//Graphs.nv(g.s_g)) - 1) * Graphs.nv(g.s_g) for
            n in Graphs.neighbors(g.s_g, v % Graphs.nv(g.s_g)) if
            (n + ((v//Graphs.nv(g.s_g)) - 1) * Graphs.nv(g.s_g) != v)
        ]
    else
        inneighbors = []
    end
    for edge in g.rem_e
        if dst(edge) == v
            filter(x -> x != src(edge), inneighbors)
        end
    end
    return inneighbors
end

function Graphs.outneighbors(g::TimeExpandedGraph, v::Int)
    nv_sg = Graphs.nv(g.s_g)
    time = (v - 1) ÷ nv_sg
    orig_v = v % nv_sg == 0 ? nv_sg : v % nv_sg

    outneighbors = Int[]

    for n in neighbors(g.s_g, orig_v)
        if (time + 1) < g.t
            dst_vertex = n + (time + 1) * nv_sg
            if dst_vertex in g.rem_v || dst_vertex == v
                continue
            end

            edge = SimpleWeightedEdge(v, dst_vertex, weights(g.s_g)[orig_v, n])
            if edge in g.rem_e
                continue
            end

            push!(outneighbors, dst_vertex)
        end
    end
    for edge in g.rem_e
        if src(edge) == v
            filter(x -> x != dst(edge), outneighbors)
        end
    end
    return outneighbors
end

function Graphs.ne(g::TimeExpandedGraph)
    return length(edges(g))
end