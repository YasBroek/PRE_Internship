using Revise
using MAPF_code
using MultiAgentPathFinding
using Graphs

mapa = "MAPF_code/input/room-32-32-4/training/room-32-32-4.map"

file_instance = readlines(mapa)
instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-5.scen")
)
my_instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, 20)
instance = "room-32-32-4"
scen_type = "random"
type_id = 13
agents = 60
scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
bench_mapf = MAPF(scen; allow_diagonal_moves=false)
cooperative_astar(bench_mapf, collect(1:agents); max_nodes=nv(bench_mapf.graph)^2)

benchmark_solution_best = Solution(scen)

test_instance = "room-32-32-4"
test_scen_type = "even"
test_type_id = 5
test_agents = 10
test_scen = BenchmarkScenario(;
    instance, scen_type=test_scen_type, type_id=test_type_id, agents=test_agents
)
test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)

weight_list, losses = MAPF_code.training_weights_gdalle(
    [bench_mapf], [benchmark_solution_best], 0.001, 10, 0.01, 200, test_bench_mapf
)

MAPF_code.visualize_edge_weights(file_instance, my_instance, weight_list)

MAPF_code.path_cost(my_instance, MAPF_code.prioritized_planning_v2(my_instance))

MAPF_code.path_cost(
    my_instance,
    MAPF_code.prioritized_planning_v2(MAPF_code.adapt_weights(my_instance, weight_list)),
)