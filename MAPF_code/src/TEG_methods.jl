Graphs.edgetype(::TimeExpandedGraph) = SimpleWeightedEdge{Int}
Graphs.has_edge(g::TimeExpandedGraph, e::SimpleWeightedEdge) = e in edges(g)
Graphs.has_vertex(g::TimeExpandedGraph, v::Int) = v in vertices(g)
Graphs.nv(g::TimeExpandedGraph) = Graphs.nv(g.s_g) * g.t
Graphs.is_directed(g::TimeExpandedGraph) = true
Graphs.is_directed(::Type{<:TimeExpandedGraph}) = true
Graphs.neighbors(g::TimeExpandedGraph, v::Int) = outneighbors(g, v)

function Graphs.weights(g::TimeExpandedGraph)
    n = nv(g)
    W = spzeros(Float64, n, n)

    for e in edges(g)
        u, v = src(e), dst(e)

        orig_u = (u % nv(g.s_g) == 0 ? nv(g.s_g) : u % nv(g.s_g))
        orig_v = (v % nv(g.s_g) == 0 ? nv(g.s_g) : v % nv(g.s_g))

        W[u, v] = Graphs.weights(g.s_g)[orig_u, orig_v]
    end

    return W
end

function Graphs.edges(g::TimeExpandedGraph)
    edge_list = []
    for v in Graphs.vertices(g.s_g)
        for n in Graphs.neighbors(g.s_g, v)
            for i in 1:(g.t - 1)
                if !((Graphs.nv(g.s_g) * i + n) in g.rem)
                    push!(
                        edge_list,
                        SimpleWeightedEdge(
                            Graphs.nv(g.s_g) * (i - 1) + v,
                            Graphs.nv(g.s_g) * i + n,
                            Graphs.weights(g.s_g)[v, n],
                        ),
                    )
                end
            end
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
    if v in g.rem
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
    return inneighbors
end

function Graphs.outneighbors(g::TimeExpandedGraph, v::Int)
    nv_sg = Graphs.nv(g.s_g)
    time = (v - 1) ÷ nv_sg
    outneighbors = [
        n + (time + 1) * nv_sg for
        n in neighbors(g.s_g, v % nv_sg == 0 ? nv_sg : v % nv_sg) if (time + 1) < g.t &&
        !(n + (time + 1) * nv_sg in g.rem) &&
        n + (time + 1) * nv_sg != v
    ]
    return outneighbors
end

function Graphs.ne(g::TimeExpandedGraph)
    return length(edges(g))
end