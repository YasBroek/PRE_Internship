using Revise
using MAPF_code
using MultiAgentPathFinding
using Graphs

instance = "room-32-32-4"
scen_type = "even"
instance_list = []
best_solutions_list = []

i = 0
for type_id in 1:25
    agents = 60
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
    bench_mapf = MAPF(scen; allow_diagonal_moves=false)
    got_it = false
    while !got_it
        try
            cooperative_astar(bench_mapf, collect(1:agents))
            got_it = true
        catch e
            agents = agents - 1
            scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
            bench_mapf = MAPF(scen; allow_diagonal_moves=false)
        end
    end
    benchmark_solution_best = Solution(scen)
    push!(instance_list, bench_mapf)
    push!(best_solutions_list, benchmark_solution_best)
    i += 1
    println(i)
end

test_instance = "room-32-32-4"
test_scen_type = "random"
test_type_id = 5
test_agents = 10
test_scen = BenchmarkScenario(;
    instance, scen_type=test_scen_type, type_id=test_type_id, agents=test_agents
)
test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)

weight_list, losses = MAPF_code.training_weights_gdalle(
    instance_list, best_solutions_list, 0.001, 10, 0.01, 200, test_bench_mapf
)
impact = 0
for type_id in 1:5
    for agents in [5, 10, 15, 20]
        a = sum_of_costs(cooperative_astar(test_bench_mapf, collect(1:10)), test_bench_mapf)
        b = sum_of_costs(
            cooperative_astar(
                MAPF_code.adapt_weights(test_bench_mapf, weight_list), collect(1:10)
            ),
            test_bench_mapf,
        )
        impact += b / a
    end
end
impact / 20

MAPF_code.visualize_edge_weights(file_instance, my_instance, weight_list)

MAPF_code.path_cost(my_instance, MAPF_code.prioritized_planning_v2(my_instance))

MAPF_code.path_cost(
    my_instance,
    MAPF_code.prioritized_planning_v2(MAPF_code.adapt_weights(my_instance, weight_list)),
)

mapa = "MAPF_code/input/empty-8-8/instance/empty-8-8.map"

file_instance = readlines(mapa)
instance_data = readlines(open("MAPF_code/input/empty-8-8/instance/empty-8-8-even-5.scen"))
my_instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, 20)
instance = "empty-8-8"
scen_type = "random"
type_id = 13
agents = 7
scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
bench_mapf = MAPF(scen; allow_diagonal_moves=false)

benchmark_solution_best = Solution(scen)

@info benchmark_solution_best

@info MAPF_code.path_to_binary_vector_gdalle(bench_mapf, benchmark_solution_best)

@info MAPF_code.Solution_to_paths(benchmark_solution_best, bench_mapf)

@info edges(benchmark_solution_best)