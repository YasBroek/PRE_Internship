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
    solution_indep = cooperative_astar(mapf, collect(1:2))
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

using CairoMakie

# Define the grid
grid = Bool[
    1 0 1 1 1
    1 0 0 0 1
    1 0 1 0 1
    1 0 0 0 1
    1 0 1 1 1
    1 0 1 1 1
]

# Define coordinates
x1 = (5, 2)
x2 = (2, 3)
y1 = (1, 2)
y2 = (6, 2)
departure_coords = [x1, y1]
arrival_coords = [x2, y2]

# Create figure
fig = Figure(; size=(600, 600))
ax = Axis(
    fig[1, 1];
    title="Grid with Departure and Arrival Points",
    xlabel="Column",
    ylabel="Row",
    aspect=DataAspect(),
    xticks=1:size(grid, 2),
    yticks=1:size(grid, 1),
)

# Display the grid as a heatmap
hm = heatmap!(ax, grid; colormap=:grays, colorrange=(0, 1))

# Define colors for each departure-arrival pair
colors = [:red, :blue]

# Plot departure coordinates as circles
for (i, coord) in enumerate(departure_coords)
    col, row = coord
    scatter!(
        ax,
        [col],
        [row];
        color=colors[i],
        markersize=20,
        marker=:circle,
        strokewidth=2,
        strokecolor=:black,
    )
end

# Plot arrival coordinates as X marks
for (i, coord) in enumerate(arrival_coords)
    col, row = coord
    scatter!(
        ax, [col], [row]; color=colors[i], markersize=20, marker=:xcross, strokewidth=3
    )
end

# Add legend
elements = [
    MarkerElement(;
        color=colors[1], marker=:circle, markersize=15, strokecolor=:black, strokewidth=1
    ),
    MarkerElement(; color=colors[1], marker=:xcross, markersize=15),
    MarkerElement(;
        color=colors[2], marker=:circle, markersize=15, strokecolor=:black, strokewidth=1
    ),
    MarkerElement(; color=colors[2], marker=:xcross, markersize=15),
]

labels = [
    "Departure 1 ($(departure_coords[1]))",
    "Arrival 1 ($(arrival_coords[1]))",
    "Departure 2 ($(departure_coords[2]))",
    "Arrival 2 ($(arrival_coords[2]))",
]

Legend(fig[1, 2], elements, labels, "Points")

# Adjust layout
colsize!(fig.layout, 1, Relative(0.7))
fig

# Define the solution paths (linear indices)
solution_paths = [[5, 4, 8, 11, 10, 9, 7], [1, 2, 3, 4, 5, 6]]

# Function to convert linear index to (column, row) coordinates
function linear_to_coord(linear_idx, grid_size)
    rows, cols = grid_size
    row = ((linear_idx - 1) ÷ cols) + 1
    col = ((linear_idx - 1) % cols) + 1
    return (col, row)
end
path_coords1 = [
    [(5, 2), (4, 2), (4, 3), (4, 4), (3, 4), (2, 4), (2, 3)],
    [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2)],
]
path_coords_ISP = [
    [(5, 2), (4, 2), (3, 2), (2, 2), (2, 3)],
    [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2)],
]
path_coords_PP = [
    [(5, 2), (4, 2), (3, 2), (2, 2), (2, 3)],
    [(1, 2), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (4, 3), (4, 2), (5, 2), (6, 2)],
]
# Plot paths for each agent
for (agent_idx, path) in enumerate(solution_paths)
    # Convert linear indices to coordinates
    path_coords = path_coords_PP[agent_idx]

    # Extract x and y coordinates for plotting
    x_coords = [coord[1] for coord in path_coords]
    y_coords = [coord[2] for coord in path_coords]

    # Plot the path as a line
    lines!(ax, x_coords, y_coords; color=colors[agent_idx], linewidth=3, linestyle=:solid)

    # Add small circles at each waypoint along the path
    scatter!(
        ax,
        x_coords[2:(end - 1)],
        y_coords[2:(end - 1)];
        color=colors[agent_idx],
        markersize=8,
        marker=:circle,
        alpha=0.6,
    )
end

# Display the figure
fig

function get_max_incoming_weights(mapf, grid_size)
    rows, cols = grid_size
    max_weights = zeros(Float64, rows, cols)

    # Get the weight matrix from the graph
    weights_matrix = mapf.graph.weights

    # Convert linear index to (row, col) coordinates
    function linear_to_rowcol(linear_idx, grid_size)
        rows, cols = grid_size
        row = ((linear_idx - 1) ÷ cols) + 1
        col = ((linear_idx - 1) % cols) + 1
        return (row, col)
    end

    # For each vertex, find the maximum weight of incoming edges
    for dest_vertex in 1:size(weights_matrix, 2)
        dest_row, dest_col = linear_to_rowcol(dest_vertex, grid_size)

        # Skip if this cell is blocked (grid value is 0)
        if !grid[dest_row, dest_col]
            max_weights[dest_row, dest_col] = NaN
            continue
        end

        max_weight = 0.0
        for src_vertex in 1:size(weights_matrix, 1)
            if weights_matrix[src_vertex, dest_vertex] > 0
                max_weight = max(max_weight, weights_matrix[src_vertex, dest_vertex])
            end
        end
        max_weights[dest_row, dest_col] = max_weight
    end

    return max_weights
end

# Create figure with subplots
fig = Figure(; size=(1200, 600))

# Left subplot: Original grid with paths
ax1 = Axis(
    fig[1, 1];
    title="Grid with Paths",
    xlabel="Column",
    ylabel="Row",
    aspect=DataAspect(),
    xticks=1:size(grid, 2),
    yticks=1:size(grid, 1),
)

# Right subplot: Edge weights heatmap
ax2 = Axis(
    fig[1, 2];
    title="Maximum Incoming Edge Weights",
    xlabel="Column",
    ylabel="Row",
    aspect=DataAspect(),
    xticks=1:size(grid, 2),
    yticks=1:size(grid, 1),
)

# Display the grid as a heatmap in left subplot
hm1 = heatmap!(ax1, grid; colormap=:grays, colorrange=(0, 1))

# Define colors for each departure-arrival pair
colors = [:red, :blue]

# Plot departure coordinates as circles
for (i, coord) in enumerate(departure_coords)
    col, row = coord
    scatter!(
        ax1,
        [col],
        [row];
        color=colors[i],
        markersize=20,
        marker=:circle,
        strokewidth=2,
        strokecolor=:black,
    )
end

# Plot arrival coordinates as X marks
for (i, coord) in enumerate(arrival_coords)
    col, row = coord
    scatter!(
        ax1, [col], [row]; color=colors[i], markersize=20, marker=:xcross, strokewidth=3
    )
end

theta_learned = -minus_theta
theta_projected = max.(theta_learned, 1e-3)
learned_graph = replace_weights(mapf.graph, theta_projected)
learned_mapf = MAPF(
    learned_graph,
    mapf.departures,
    mapf.arrivals;
    vertex_conflicts=mapf.vertex_conflicts,
    edge_conflicts=mapf.edge_conflicts,
)

max_weights = get_max_incoming_weights(learned_mapf, size(grid))

hm2 = heatmap!(ax2, max_weights; colormap=:viridis, nan_color=:gray)

# Add colorbar for the weights
Colorbar(fig[1, 3], hm2; label="Max Incoming Edge Weight")

# Add text annotations showing the weight values
for i in 1:size(grid, 1), j in 1:size(grid, 2)
    if grid[i, j]  # Only for passable cells
        weight_val = max_weights[i, j]
        if !isnan(weight_val)
            text!(
                ax2,
                j,
                i;
                text=string(round(weight_val; digits=2)),
                color=:white,
                fontsize=10,
                align=(:center, :center),
            )
        end
    end
end

# Add legend for the first subplot
elements = [
    MarkerElement(;
        color=colors[1], marker=:circle, markersize=15, strokecolor=:black, strokewidth=1
    ),
    MarkerElement(; color=colors[1], marker=:xcross, markersize=15),
    MarkerElement(;
        color=colors[2], marker=:circle, markersize=15, strokecolor=:black, strokewidth=1
    ),
    MarkerElement(; color=colors[2], marker=:xcross, markersize=15),
]

labels = [
    "Departure 1 ($(departure_coords[1]))",
    "Arrival 1 ($(arrival_coords[1]))",
    "Departure 2 ($(departure_coords[2]))",
    "Arrival 2 ($(arrival_coords[2]))",
]

Legend(fig[2, 1:2], elements, labels, "Points"; orientation=:horizontal)

# Display the figure
fig