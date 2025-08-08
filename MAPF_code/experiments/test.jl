using Revise
using MAPF_code
using MultiAgentPathFinding
using Graphs
using CairoMakie
using Statistics
using MultiAgentPathFinding: NoConflictFreePathError

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
i = 0
for type_id in 1:25
    for agents in 1:40
        scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
        bench_mapf = MAPF(scen; allow_diagonal_moves=false)
        benchmark_solution_best = Solution(scen)
        push!(instance_list, bench_mapf)
        push!(best_solutions_list, benchmark_solution_best)
        i += 1
        println(i)
    end
end

test_instance = "room-32-32-4"
test_scen_type = "random"
test_type_id = 5
test_agents = 45
test_scen = BenchmarkScenario(;
    instance, scen_type=test_scen_type, type_id=test_type_id, agents=test_agents
)
test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)

weight_list, losses = MAPF_code.training_weights_gdalle(
    instance_list, best_solutions_list, 0.1, 10, 0.01, 200, test_bench_mapf
)
impact = 0
for type_id in 1:5
    for agents in [5, 10, 15, 20]
        test_scen = BenchmarkScenario(; instance, scen_type=test_scen_type, type_id, agents)
        test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)
        a = sum_of_costs(
            cooperative_astar(test_bench_mapf, collect(1:agents)), test_bench_mapf
        )
        b = sum_of_costs(
            cooperative_astar(
                MAPF_code.adapt_weights(test_bench_mapf, weight_list), collect(1:agents)
            ),
            test_bench_mapf,
        )
        impact += b / a
    end
end
impact / 20

impact_list = []
for agents in 1:50
    impact = []
    for type_id in 1:10
        test_scen = BenchmarkScenario(; instance, scen_type=test_scen_type, type_id, agents)
        test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)
        try
            a = sum_of_costs(
                cooperative_astar(test_bench_mapf, collect(1:agents)), test_bench_mapf
            )
            b = sum_of_costs(
                cooperative_astar(
                    MAPF_code.adapt_weights(test_bench_mapf, weight_list), collect(1:agents)
                ),
                test_bench_mapf,
            )
            push!(impact, b / a)
        catch e
            continue
        end
    end
    push!(impact_list, mean(impact))
end
impact_list

agents = 1:50
fig = Figure(; resolution=(800, 500))

ax = Axis(
    fig[1, 1];
    xlabel="Number of Agents",
    ylabel="Path cost change ratio",
    title="Cost ratio vs Number of Agents",
)

lines!(ax, agents, impact_list; color=:blue, linewidth=2)
scatter!(ax, agents, impact_list; color=:red)

fig
impact_list

instance_data = readlines(
    "MAPF_code/input/room-32-32-4/instance/room-32-32-4-random-10.scen"
)
file_instance = readlines("MAPF_code/input/room-32-32-4/training/room-32-32-4.map")
my_instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, 25)

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