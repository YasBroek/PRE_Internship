"""
    function extract_features(instance::MAPF_Instance)
Extracts features from an instance for each edge

# Arguments
 - instance

# Returns
 - Matrix of edges x features
"""
function extract_features(instance::MAPF_Instance)
    edge_list = collect(edges(instance.graph))
    features = zeros(Float64, ne(instance.graph), 9)
    paths = independent_shortest_paths(instance)
    conflicts = conflict_identifier(instance, paths)
    for edge in 1:ne(instance.graph)
        features[edge, 1] = conflict_counter(conflicts, edge_list[edge])  # conflict-based feature
        features[edge, 2] = degree(instance.graph, dst(edge_list[edge]))  # degree-based feature
        features[edge, 3] = harmonic_centrality(instance, edge_list[edge]) # closeness to all other vertices
        features[edge, 4] = step_counter(paths, edge_list[edge]) # number of paths going through edge
        features[edge, 5] = distance_to_closest_obstacle(instance, edge_list[edge])
        features[edge, 6] = distance_to_all_agents(instance, edge_list[edge])
        features[edge, 7] = distance_to_closest_agent(instance, edge_list[edge])
        features[edge, 8] = number_of_agents_close(instance, edge_list[edge])
        features[edge, 9] = normalized_closeness_centrality(instance, edge_list[edge])
    end
    return features
end

"""
    function linear_regression(edge_features::Array{Int}, regression_weights::Vector{Float64})
Used to apply linear regression to a feature Matrix

# Arguments
 - edge_features: Matrix of edges x features
 - regression_weights: Calculated weights for each feature
"""
function linear_regression(
    edge_features::Array{Float64}, regression_weights::Vector{Float64}
)
    return edge_features * regression_weights
end

"""
    function adapt_weights(instance::MAPF_Instance, perturbed_θ::Vector{T})
Attributes new calculated weights to graph edges

# Arguments
 - instance: constains graph which will be adapted
 - perturbed_θ: vector containing incoming weights for edges

# Returns
 - adapted instance with new weights
"""
function adapt_weights(instance::MAPF_Instance, perturbed_θ::Vector{T}) where {T<:Real}
    num_edges = ne(instance.graph)
    length(perturbed_θ) == num_edges || throw(
        ArgumentError(
            "perturbed_θ must have length $num_edges, got $(length(perturbed_θ))"
        ),
    )
    edge_list = collect(edges(instance.graph))

    for (i, edge) in enumerate(edge_list)
        u, v = src(edge), dst(edge)
        instance.graph.weights[u, v] = perturbed_θ[i]
        instance.graph.weights[v, u] = perturbed_θ[i]
    end
    return instance
end

"""
    Function training_LR(instance::MAPF_Instance, solution_algorithm::Function, ϵ::Float64, M::Int, α::Float64, num_epochs::Int)
training function for machine learning

# Arguments
 - instance used for training
 - solution_algorithm: function that will be used for finding a solution at each iteration
 - ϵ: perturbation argument
 - M: number of iterations for Monte Carlo sampling
 - α: learning rate 
 - num_epochs: number of epochs for training
"""
function training_LR(
    instance_list, benchmarkSols, ϵ::Float64, M::Int, α::Float64, num_epochs::Int
)
    regression_weights = randn(9)
    epoch_list = [x for x in 1:num_epochs]
    loss_gradient_list = []
    loss_list = []
    features_list = []
    local y_estimate, fenchel_loss_gradient
    for epoch in 1:num_epochs
        avg_grad = 0
        avg_loss = 0
        for index in 1:length(instance_list)
            y_best_found_solution = path_to_binary_vector(
                instance_list[index],
                Solution_to_paths(benchmarkSols[index], instance_list[index]),
            )
            if epoch == 1
                push!(features_list, extract_features(instance_list[index]))
            end
            if epoch == 1 && index == 1
                y_estimate = spzeros(ne(instance_list[1].graph))
            end
            fill!(y_estimate, 0.0)

            θ = linear_regression(features_list[index], regression_weights)
            for m in 1:M
                Z_m = randn(size(θ))
                perturbed_θ = θ .+ ϵ .* Z_m
                weighted_instance = adapt_weights(instance_list[index], perturbed_θ)
                y_estimate += path_to_binary_vector(
                    weighted_instance, independent_shortest_paths(weighted_instance)
                )
            end
            y_estimate = y_estimate ./ M
            fenchel_loss_gradient = y_best_found_solution - y_estimate
            regression_weights =
                regression_weights - α * features_list[index]' * fenchel_loss_gradient
            fenchel_loss = fenchel_young_loss(θ, y_best_found_solution)
            println("epoch: $epoch")
            avg_grad += sum(abs.(fenchel_loss_gradient))
            avg_loss += fenchel_loss
        end
        avg_grad = avg_grad / length(instance_list)
        avg_loss = avg_loss / length(instance_list)
        push!(loss_gradient_list, avg_grad)
        push!(loss_list, avg_loss)
    end
    display(
        lineplot(
            epoch_list,
            loss_gradient_list;
            title="Gradient loss over time",
            name="my line",
            xlabel="epoch",
            ylabel="loss",
        ),
    )
    display(
        lineplot(
            epoch_list,
            loss_list;
            title="Loss over time",
            name="my line",
            xlabel="epoch",
            ylabel="loss",
        ),
    )
    return regression_weights
end

function calculate_path(instance, regression_weights)
    features = extract_features(instance)
    weighted_instance = deepcopy(instance)
    θ = linear_regression(features, regression_weights)

    weighted_instance = adapt_weights(weighted_instance, θ)

    paths_vertices = cooperative_astar(
        MAPF(weighted_instance.graph, instance.starts, instance.goals),
        collect(1:length(instance.starts)),
    )
    paths = [
        [
            SimpleWeightedEdge(
                paths_vertices.paths[k][i],
                paths_vertices.paths[k][i + 1],
                weights(instance.graph)[
                    paths_vertices.paths[k][i], paths_vertices.paths[k][i + 1]
                ],
            ) for i in 1:(length(paths_vertices.paths[k]) - 1)
        ] for k in 1:length(paths_vertices.paths)
    ]

    return paths
end

function calculate_path_v(instance, regression_weights)
    features = extract_features(instance)
    weighted_instance = deepcopy(instance)
    θ = linear_regression(features, regression_weights)

    weighted_instance = adapt_weights(weighted_instance, θ)

    paths_vertices = cooperative_astar(
        MAPF(weighted_instance.graph, instance.starts, instance.goals),
        collect(1:length(instance.starts)),
    )
    return paths_vertices
end

function fenchel_young_loss(θ::Vector{Float64}, y_target)
    # Regularização conjugada: Ω*(θ) = ½‖θ‖²
    Ω_star = 0.5 * sum(θ .^ 2)
    return Ω_star - dot(θ, y_target)
end