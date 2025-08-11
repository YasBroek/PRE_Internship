using MultiAgentPathFinding  # version from main branch
using MultiAgentPathFinding: replace_weights, vectorize_weights
using SimpleWeightedGraphs
using StableRNGs
using UnicodePlots
using InferOpt
using Zygote
using Revise
using MAPF_code
using SparseArrays
using LinearAlgebra
using Statistics

function my_sum_of_costs(Solution, mapf)
    cost = 0
    for path in Solution.paths
        for v in 1:(length(path) - 1)
            cost += mapf.graph.weights[path[v], path[v + 1]]
        end
    end
    return cost
end

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

function maximizer(minus_theta; mapf)
    theta = -minus_theta  # independent_dijkstra is a *minimizer*
    theta_projected = max.(theta, 1e-3)  # ensure positive weights
    new_graph = replace_weights(mapf.graph, theta_projected)
    new_mapf = MAPF(
        new_graph,
        mapf.departures,
        mapf.arrivals;
        vertex_conflicts=mapf.vertex_conflicts,
        edge_conflicts=mapf.edge_conflicts,
    )
    solution_indep = independent_dijkstra(new_mapf)
    y = count_edge_visits(solution_indep, mapf)
    return y
end

grid = Bool[
    1 0 1 1 1
    1 0 0 0 1
    1 0 1 0 1
    1 0 0 0 1
    1 0 1 1 1
    1 0 1 1 1
]
x1 = (5, 2)
x2 = (2, 3)
y1 = (1, 2)
y2 = (6, 2)
departure_coords = [x1, y1]
arrival_coords = [x2, y2]
mapf = MAPF(grid, departure_coords, arrival_coords; allow_diagonal_moves=false)
_, coord_to_vertex, vertex_to_coord = parse_benchmark_map(grid; allow_diagonal_moves=false)

cooperative_astar(mapf, collect(1:2))

optimal_solution = Solution([[5, 4, 8, 11, 10, 9, 7], [1, 2, 3, 4, 5, 6]])  # by accident
y_opt = count_edge_visits(optimal_solution, mapf)

nb_samples = 10
ε = 1e-1
learning_rate = 1e-2

layer = PerturbedAdditive(maximizer; ε, nb_samples, seed=0, rng=StableRNG(0), threaded=true)
loss = FenchelYoungLoss(layer)

minus_theta = -vectorize_weights(mapf.graph)
losses = Float64[]

for epoch in 1:200
    @info "epoch $epoch"
    yield()

    l, grads = Zygote.withgradient(mt -> loss(mt, y_opt; mapf), minus_theta)
    grad = grads[1]

    minus_theta -= learning_rate * grad
    minus_theta = min.(minus_theta, -1e-3)
    push!(losses, l)
end

lineplot(eachindex(losses), losses)

sum_of_costs(cooperative_astar(mapf, collect(1:2)), mapf)
theta = -minus_theta  # independent_dijkstra is a *minimizer*
theta_projected = max.(theta, 1e-3)  # ensure positive weights
new_graph = replace_weights(mapf.graph, theta_projected)
new_mapf = MAPF(
    new_graph,
    mapf.departures,
    mapf.arrivals;
    vertex_conflicts=mapf.vertex_conflicts,
    edge_conflicts=mapf.edge_conflicts,
)
sum_of_costs(cooperative_astar(new_mapf, collect(1:2)), mapf)

weight_grid = fill(NaN, 6, 5)

actual_weights = -minus_theta
edge_list = collect(Graphs.edges(mapf.graph))
weights_vertices = [Vector{Float64}() for _ in 1:nv(mapf.graph)]
for (edge_idx, e) in enumerate(edge_list)
    dst_vertex = dst(e)
    push!(weights_vertices[dst_vertex], actual_weights[edge_idx])
end
mean_weights_vertices = [isempty(w) ? NaN : maximum(w) for w in weights_vertices]

for v in Graphs.vertices(mapf.graph)
    coords = vertex_to_coord[v]
    weight_grid[Int(coords[1]), Int(coords[2])] = mean_weights_vertices[v]
end

fig = Figure(; resolution=(800, 800))
ax = Axis(fig[1, 1]; aspect=DataAspect(), title="Learned Edge Weights")

hm = heatmap!(ax, weight_grid'; colormap=:plasma, nan_color=:black)

Colorbar(fig[1, 2], hm; label="Edge Weight")
fig

using CairoMakie, Colors

fig = Figure(; resolution=(800, 800))
ax = Axis(fig[1, 1]; aspect=DataAspect(), title="Grid Original")

# Inverter valores para 1=preto e 0=branco
hm = heatmap!(ax, Float64.(grid'); colormap=[RGB(1, 1, 1), RGB(0, 0, 0)])

scatter!(
    ax,
    [s[2] for s in departure_coords],
    [s[1] for s in departure_coords];
    color=colors[3:4],
    marker=[:star4, :utriangle],
    markersize=40,
)

scatter!(
    ax,
    [s[2] for s in arrival_coords],
    [s[1] for s in arrival_coords];
    color=colors[1:2],
    marker=[:star4, :utriangle],
    markersize=40,
)
fig