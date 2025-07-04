using Revise
using MAPF_code
using Graphs

"Open map"
file_instance = readlines(open("MAPF_code/input/Berlin_1_256/instance/Berlin_1_256.map"))

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/Berlin_1_256/instance/Berlin_1_256-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 1

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, 4)
new_instance = MAPF_code.modify_agent_quantity(instance_data, instance, 4)

"Open solution"
solutions = readlines(open("input/Berlin_1_256/solution/Berlin_1_256.csv"))
solutions_header = split(solutions[1], ",")
lower_costs = Dict{Tuple{String,Int,Int},Float64}()
for line in solutions[2:end]
    values = split(line, ",")
    scen_type = strip(values[1], '"')
    type_id = parse(Int, strip(values[2], '"'))
    agents = parse(Int, strip(values[3], '"'))
    lower_cost = parse(Float64, strip(values[4], '"'))

    key = (scen_type, type_id, agents)
    lower_costs[key] = lower_cost
end

instance_solution = lower_costs[(instance_scen_type, instance_type_id, num_agents)]
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

MAPF_code.visualization(
    file_instance, instance, MAPF_code.independent_shortest_paths(instance)
)

MAPF_code.training_LR(instance, MAPF_code.independent_shortest_paths, 0.5, 10, 0.001, 50)

@profview for _ in 1:10
    MAPF_code.training_LR(
        instance, MAPF_code.independent_shortest_paths, 0.5, 10, 0.001, 10
    )
end

MAPF_code.path_cost(MAPF_code.independent_shortest_paths(instance))

@info MAPF_code.independent_shortest_paths(instance)

MAPF_code.training_LR(instance, MAPF_code.independent_shortest_paths, 0.5, 10, 0.001, 10)

l = 0

for path in MAPF_code.independent_shortest_paths(instance)
    for edge in path
        l += 1
    end
end

l