"""
	struct MAPF_Instance: Multi-Agent Path Finding (MAPF) problem instance.

# Fields
 - 'graph::SimpleGraph': Graph of the environment
 - 'starts::Vector': Starting positions of agents 
 - 'goals::Vector': Goal positions of agents
 - 'optimal_values::Vector': Optimal path costs for each agent.
	"""
mutable struct MAPF_Instance
    height::Int
    width::Int
    graph::SimpleWeightedGraph
    starts::Vector
    goals::Vector
    optimal_values::Vector
    scenario_numbers::Vector{Int}
end

"""
struct MAPF_Solution: Solution found by the algorithm
# Fields
	- 'paths::Vector': lists all the positions occupied throught the simulation at each time step for each agent
"""
struct MAPF_Solution
    paths::Vector
end

struct TimeState
    vertex::Int
    time_step::Int
end

mutable struct TimeExpandedGraph <: AbstractGraph{Int}
    s_g::SimpleWeightedGraph
    t::Int
    rem_v::Vector{Int}
    rem_e::Vector{SimpleWeightedEdge{Int,Float64}}
    goal::Int
end

function TimeExpandedGraph(
    s_g::SimpleWeightedGraph{Int64,Float64},
    t::Int,
    rem_v::Vector{Int},
    rem_e::Vector{SimpleWeightedEdge{Int,Float64}},
)
    return TimeExpandedGraph(s_g, t, rem_v, rem_e, 0)
end

struct TimeExpandedWeights{T}
    W_sg::SparseMatrixCSC{T,Int}
end

Base.getindex(w::TimeExpandedWeights, i::Int, j::Int) = w.W[i, j]
Base.size(w::TimeExpandedWeights) = size(w.W)