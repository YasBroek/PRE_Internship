"""
	struct MAPF_Instance: Multi-Agent Path Finding (MAPF) problem instance.

# Fields
 - 'graph::SimpleGraph': Graph of the environment
 - 'starts::Vector': Starting positions of agents 
 - 'goals::Vector': Goal positions of agents
 - 'optimal_values::Vector': Optimal path costs for each agent.
	"""
struct MAPF_Instance
    height::Int
    width::Int
    graph::SimpleWeightedGraph
    starts::Vector
    goals::Vector
    optimal_values::Vector
    scenario_numbers::Vector{Int}
    y_optimum::Float64
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

mutable struct TimeExpandedGraph
    s_g::SimpleWeightedGraph
    t::Int
    rem::Vector{Int}
end
