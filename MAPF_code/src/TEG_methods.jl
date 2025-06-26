edgetype(::TimeExpandedGraph) = SimpleWeightedEdge{Int}
has_edge(g::TimeExpandedGraph, e::SimpleWeightedEdge) = e in edges(g) ? true : false
has_vertex(g::TimeExpandedGraph, v::Int) = v in vertices(g) ? true : false
nv(g::TimeExpandedGraph) = Graphs.nv(g.s_g) * g.t - length(unique(g.rem))
is_directed(g::TimeExpandedGraph) = true
is_directed(::Type{<:TimeExpandedGraph}) = true
neighbors(g::TimeExpandedGraph, v::Int) = outneighbors(g, v)
weight(g::TimeExpandedGraph, u::Int, v::Int) = begin
    for e in edges(g)
        if src(e) == u && dst(e) == v
            return weight(e)
        end
    end
    return Inf
end

function edges(g::TimeExpandedGraph)
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

function vertices(g::TimeExpandedGraph)
    vertex_list = [
        Graphs.nv(g.s_g) * (i - 1) + v for v in 1:Graphs.nv(g.s_g) for i in 1:(g.t)
    ]
    for item in g.rem
        filter!(x -> x != item, vertex_list)
    end
    return vertex_list
end

function inneighbors(g::TimeExpandedGraph, v::Int)
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

function outneighbors(g::TimeExpandedGraph, v::Int)
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

function ne(g::TimeExpandedGraph)
    return length(edges(g))
end