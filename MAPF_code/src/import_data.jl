"""
	coords_to_index(coord, width) and index_to_coords(index, width)
	
Converts data format from coordinates (x, y) (grid format) to index (graph format) (and vice-versa)
"""
function coords_to_index(coord, width)
    index = (coord[2] - 1) * width + coord[1]
    return index
end

function index_to_coords(index, width)
    x = (index % width == 0 ? width : index % width)
    y = (index % width == 0 ? index / width : div(index, width) + 1)
    return (x, y)
end

"""
    convert_to_my_struct(file_instance, instance_data, num_agents, instance_solution)

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
        error(
            "Requested number of agents ($num_agents) exceeds available agents ($max_agents)",
        )
    end

    # Parse the height and width of the grid from the file
    height = parse(Int, split(file_instance[2])[2])
    width = parse(Int, split(file_instance[3])[2])

    # Initialize the MAPF_Instance with a graph and vectors for start/goal/optimal values
    instance = MAPF_Instance(
        height,
        width,
        SimpleWeightedGraph(height * width),
        Vector{Int}(undef, num_agents),
        Vector{Int}(undef, num_agents),
        Vector{Float64}(undef, num_agents),
        Vector{Int}(undef, num_agents),
    )

    # Fill in the graph based on map characters: '.' = possible direction, otherwise obstacle
    for i in 1:height
        row = file_instance[i + 4]
        for j in 1:width
            if row[j] == '.'
                for (di, dj) in ((0, 1), (1, 0), (0, 0))
                    ni = i + di
                    nj = j + dj
                    if 1 <= ni <= height && 1 <= nj <= width
                        if file_instance[ni + 4][nj] == '.'
                            add_edge!(
                                instance.graph,
                                coords_to_index((j, i), width),
                                coords_to_index((nj, ni), width),
                                1.0,
                            )
                        end
                    end
                end
                for (di, dj) in ((1, 1), (-1, 1))
                    ni = i + di
                    nj = j + dj
                    if 1 <= ni <= height && 1 <= nj <= width
                        if file_instance[ni + 4][j] == "." &&
                            file_instance[i + 4][nj] == "."
                            add_edge!(
                                instance.graph,
                                coords_to_index((j, i), width),
                                coords_to_index((nj, ni), width),
                                sqrt(2),
                            )
                        end
                    end
                end
            end
        end
    end

    # Parse start, goal and optimal value for each agent from Scenario Instance
    for i in 1:num_agents
        row = instance_data[i + 1]
        fields = split(row)

        start_x = parse(Int, fields[5]) + 1
        start_y = parse(Int, fields[6]) + 1
        goal_x = parse(Int, fields[7]) + 1
        goal_y = parse(Int, fields[8]) + 1

        instance.scenario_numbers[i] = parse(Int, fields[1])
        instance.starts[i] = coords_to_index((start_x, start_y), width)
        instance.goals[i] = coords_to_index((goal_x, goal_y), width)
        instance.optimal_values[i] = parse(Float64, fields[9])
    end

    return instance
end

function increase_agent_quantity(
    instance_data, instance::MAPF_Instance, new_number_of_agents::Int
)
    old_number_of_agents = length(instance.starts)
    if old_number_of_agents < new_number_of_agents
        for i in (old_number_of_agents + 1):new_number_of_agents
            row = instance_data[i + 1]
            fields = split(row)

            start_x = parse(Int, fields[5]) + 1
            start_y = parse(Int, fields[6]) + 1
            goal_x = parse(Int, fields[7]) + 1
            goal_y = parse(Int, fields[8]) + 1

            push!(instance.scenario_numbers, parse(Int, fields[1]))
            push!(instance.starts, coords_to_index((start_x, start_y), instance.width))
            push!(instance.goals, coords_to_index((goal_x, goal_y), instance.width))
            push!(instance.optimal_values, parse(Float64, fields[9]))
        end
    else
        print(
            "Warning: new number of agents is lesser or equal to previous: instance unchanged",
        )
    end
    return instance
end

function change_scenarios(instance_data, instance::MAPF_Instance, num_agents::Int)
    new_instance = MAPF_Instance(
        instance.height,
        instance.width,
        instance.graph,
        Vector{Int}(undef, num_agents),
        Vector{Int}(undef, num_agents),
        Vector{Float64}(undef, num_agents),
        Vector{Int}(undef, num_agents),
    )
    for i in 1:num_agents
        row = instance_data[i + 1]
        fields = split(row)

        start_x = parse(Int, fields[5]) + 1
        start_y = parse(Int, fields[6]) + 1
        goal_x = parse(Int, fields[7]) + 1
        goal_y = parse(Int, fields[8]) + 1

        new_instance.scenario_numbers[i] = parse(Int, fields[1])
        new_instance.starts[i] = coords_to_index((start_x, start_y), instance.width)
        new_instance.goals[i] = coords_to_index((goal_x, goal_y), instance.width)
        new_instance.optimal_values[i] = parse(Float64, fields[9])
    end
    return new_instance
end