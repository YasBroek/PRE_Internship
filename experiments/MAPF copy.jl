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
num_agents = 3

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

path_prioritized = MAPF_code.prioritized_planning(instance)

MAPF_code.visualization(file_instance, instance, path_prioritized)

@info MAPF_code.prioritized_planning(instance)

@info MAPF_code.independent_shortest_paths(instance)

using Graphs
println("Has path: ", has_path(instance.graph, instance.starts[2], instance.goals[2]))

@info neighbors(instance.graph, instance.starts[7])

MAPF_code.training_LR(instance, 0.5, 10, 0.001, 500)

@info MAPF_code.path_to_binary(instance, MAPF_code.independent_shortest_paths(instance))