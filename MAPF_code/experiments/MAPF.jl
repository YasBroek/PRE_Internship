using Revise
using MAPF_code
using Graphs

"Open map"
file_instance = readlines(open("input/Berlin_1_256/instance/Berlin_1_256.map"))

"Open scenarios"
instance_data = readlines(open("input/Berlin_1_256/instance/Berlin_1_256-even-1.scen"))
instance_type_id = 1
instance_scen_type = "even"
num_agents = 5

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

MAPF_code.training_LR(instance, MAPF_code.independent_shortest_paths, 0.1, 10, 0.01, 3)