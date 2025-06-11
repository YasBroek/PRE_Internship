"Open map"
file_instance = readlines(open("../input/Berlin_1_256/instance/Berlin_1_256.map"))

"Open scenarios"
instance_data = readlines(open("../input/Berlin_1_256/instance/Berlin_1_256-even-1.scen"))

"Open solution"
solutions = readlines(open("../input/Berlin_1_256/solution/Berlin_1_256.csv")) 

"""
    convert_to_my_struct(file_instance, instance_data)

Converts input data into a MAPF_Instance struct

# Arguments
- 'file_instance': contains data from map file
- 'instance_data': contains data from scenaio file

# Returns
- 'MAPF_Instance' struct

"""
function convert_to_my_struct(file_instance, instance_data, num_agents)

	max_agents = length(instance_data) - 1
    if num_agents > max_agents
        error("Requested number of agents ($num_agents) exceeds available agents ($max_agents)")
    end
	
	# Parse the height and width of the grid from the file
	height = parse(Int, split(file_instance[2])[2])
	width = parse(Int, split(file_instance[3])[2])

	# Initialize the MAPF_Instance with a graph and vectors for start/goal/optimal values
	instance = MAPF_Instance(
		height, 
		width, 
		SimpleGraph(height * width), 
		Vector{Int}(undef,num_agents), 
		Vector{Int}(undef,num_agents), 
		Vector{Float64}(undef,num_agents),
		Vector{Int}(undef, num_agents)
	)

	# Fill in the graph based on map characters: '.' = possible direction, otherwise obstacle
	for i in 1:height
		row = file_instance[i + 4]
        for j in 1:width
            if row[j] == '.'
				for (di, dj) in ((0,1),(1,0))
					ni = i + di
					nj = j + dj
					if 1 <= ni <= height && 1 <= nj <= width
						if file_instance[ni+4][nj] == '.'
							add_edge!(instance.graph, coords_to_index((i,j), width), coords_to_index((ni,nj),width))
						end
					end
				end
            end
        end
	end

	# Parse start, goal and optimal value for each agent from Scenario Instance
	for i in 1:num_agents
		row = instance_data[i+1]
		fields = split(row)

		start_x = parse(Int, fields[5]) + 1
		start_y = parse(Int, fields[6]) + 1  
        goal_x = parse(Int, fields[7]) + 1  
        goal_y = parse(Int, fields[8]) + 1
		
		instance.scenario_numbers[i] = parse(Int, fields[1])
		instance.starts[i] = coords_to_index((start_x,start_y),width)
		instance.goals[i] = coords_to_index((goal_y,goal_x),width)
		instance.optimal_values[i] = parse(Float64, fields[9])
	end

	return instance
end