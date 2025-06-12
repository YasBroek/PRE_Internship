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
	graph::SimpleGraph
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

"""
	coords_to_index(coord, width) and index_to_coords(index, width)
	
Converts data format from coordinates (x, y) (grid format) to index (graph format) (and vice-versa)
"""
function coords_to_index(coord, width)
	index = (coord[2]-1)*width + coord[1]
	return index
end

function index_to_coords(index, width)
	x = (index - 1) % width + 1
    y = (index - 1) ÷ width + 1
    return (x, y)
end


