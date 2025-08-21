using Revise
using MAPF_code
using MultiAgentPathFinding
using Graphs
using CairoMakie
using Statistics

instance = "room-32-32-4"
no_conflict_instances = []
PP = 0
special = []
best_solutions_list = []
special_best_solutions = []

for scen_type in ["even", "random"]
    for type_id in 1:25
        for agents in 1:60
            scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
            bench_mapf = MAPF(scen; allow_diagonal_moves=false)
            ISP = independent_dijkstra(bench_mapf)
            if is_feasible(ISP, bench_mapf)
                push!(no_conflict_instances, bench_mapf)
                push!(best_solutions_list, Solution(scen))
                if agents > 9
                    push!(special, bench_mapf)
                    push!(special_best_solutions, Solution(scen))
                end
            end
        end
    end
end

@info special

weight_list, losses = MAPF_code.training_weights_gdalle(
    [special[3]],
    [special_best_solutions[3]],
    0.1,
    10,
    0.01,
    250,
    rand(no_conflict_instances),
)
