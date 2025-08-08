using MultiAgentPathFinding  # version from main branch
using MultiAgentPathFinding: replace_weights, vectorize_weights
using SimpleWeightedGraphs
using StableRNGs
using UnicodePlots
using Graphs

using SparseArrays
using LinearAlgebra
using Statistics

function count_edge_visits(solution::Solution, mapf::MAPF)
    visits = similar(mapf.graph.weights, Int)
    visits.nzval .= 0
    for path in solution.paths
        for t in 1:(length(path) - 1)
            visits[path[t], path[t + 1]] += 1
            if path[t] != path[t + 1]
                visits[path[t + 1], path[t]] += 1
            end
        end
    end
    return vectorize_weights(SimpleWeightedGraph(visits))
end

grid = Bool[
    1 0 1
    1 0 1
    1 0 0
    1 0 1
    1 0 1
]
x1 = (1, 2)
x2 = (4, 2)
y1 = (5, 2)
y2 = (2, 2)
departure_coords = [x1, y1]
arrival_coords = [x2, y2]
mapf = MAPF(grid, departure_coords, arrival_coords; allow_diagonal_moves=false)
_, coord_to_vertex, vertex_to_coord = parse_benchmark_map(grid; allow_diagonal_moves=false)

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

optimal_solution = Solution([[1, 2, 3, 6, 3, 4], [5, 4, 4, 3, 2, 1]])
y_opt = count_edge_visits(optimal_solution, mapf)

nb_samples = 10
ε = 1e-1
learning_rate = 1e-2

theta = vectorize_weights(mapf.graph)
losses = Float64[]
rng = StableRNG(0)

for epoch in 1:200
    @info "epoch $epoch"
    yield()
    for (index, instance) in enumerate(instance_list)
        mapf = instance
        y_opt = count_edge_visits(best_solutions_list[index], mapf)

        y_and_l_perturbed = map(1:nb_samples) do _
            theta_perturbed = theta + ε .* randn(rng, length(theta))  # don't bother with multiplicative
            theta_perturbed_projected = max.(theta_perturbed, 1e-3)
            graph_perturbed = replace_weights(mapf.graph, theta_perturbed_projected)
            mapf_perturbed = MAPF(
                graph_perturbed,
                mapf.departures,
                mapf.arrivals;
                vertex_conflicts=mapf.vertex_conflicts,
                edge_conflicts=mapf.edge_conflicts,
            )
            solution_indep = independent_dijkstra(mapf_perturbed)
            y = count_edge_visits(solution_indep, mapf_perturbed)
            l = dot(theta_perturbed_projected, y)
            return y, l
        end
        y_avg = mean(first.(y_and_l_perturbed))
        l_avg = mean(last.(y_and_l_perturbed))

        grad = y_avg - y_opt
        theta -= learning_rate * grad
        theta = max.(theta, 1e-3)

        push!(losses, l_avg)
    end
end

histogram(theta)
lineplot(eachindex(losses), losses)

@info theta