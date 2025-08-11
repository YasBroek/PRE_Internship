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

test_instance = "room-32-32-4"
test_scen_type = "random"
test_type_id = 1
test_agents = 45
test_scen = BenchmarkScenario(;
    instance, scen_type=test_scen_type, type_id=test_type_id, agents=test_agents
)
test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)

weight_list, losses = MAPF_code.training_weights_gdalle(
    [instance_list[1]], [best_solutions_list[1]], 0.01, 2, 0.0001, 1001, test_bench_mapf
)

cost_ratios = []
num_agents = []
ids = []

for _ in 1:500
    test_instance = "room-32-32-4"
    test_scen_type = rand(["even", "random"])
    test_type_id = rand(1:25)
    test_agents = rand(1:50)
    test_scen = BenchmarkScenario(;
        instance=test_instance,
        scen_type=test_scen_type,
        type_id=test_type_id,
        agents=test_agents,
    )
    test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)
    try
        push!(cost_ratios, MAPF_code.apply_prediction(test_bench_mapf, weight_list))
        push!(num_agents, test_agents)
        push!(ids, test_type_id)
    catch e
        continue
    end
end

# Make sure they are regular Julia arrays
x1 = num_agents
x2 = ids
y = cost_ratios

fig1 = Figure()
fig2 = Figure()
ax1 = Axis(
    fig1[1, 1];
    xlabel="Number of Agents",
    ylabel="Cost Ratio",
    title="MAPF Prediction Performance",
)
ax2 = Axis(
    fig2[1, 1]; xlabel="Type id", ylabel="Cost Ratio", title="MAPF Prediction Performance"
)

scatter!(ax1, x1, y; color=:blue, markersize=8, strokewidth=0.5)
scatter!(ax2, x2, y; color=:blue, markersize=8, strokewidth=0.5)

fig1
fig2