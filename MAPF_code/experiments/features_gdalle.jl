using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
using JLD2
using InferOpt
using Flux: softplus
using Graphs
using Random
using ProgressMeter

instance = "room-32-32-4"
scen_type = "even"
instance_list = []
best_solutions_list = []

i = 0
for type_id in 1:25
    agents = rand(1:30)
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
    bench_mapf = MAPF(scen; allow_diagonal_moves=false)
    benchmark_solution_best = Solution(scen)
    push!(instance_list, bench_mapf)
    push!(best_solutions_list, benchmark_solution_best)
    i += 1
    println(i)
end

test_instance = "room-32-32-4"
test_scen_type = "random"
test_type_id = 5
test_agents = 35
test_scen = BenchmarkScenario(;
    instance, scen_type=test_scen_type, type_id=test_type_id, agents=test_agents
)
test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)

features_list = []
@showprogress for instance in instance_list
    push!(features_list, MAPF_code.extract_features_gdalle(instance))
end

weight_list, losses = MAPF_code.training_gdalle(
    instance_list, features_list, best_solutions_list, 0.1, 10, 0.01, 80, test_bench_mapf
)