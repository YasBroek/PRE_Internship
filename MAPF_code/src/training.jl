"""
    function extract_features(instance::MAPF_Instance)
Extracts features from an instance for each edge

# Arguments
 - instance

# Returns
 - Matrix of edges x features
"""
function extract_features(instance::MAPF_Instance)
    features = Array{Int}(undef, ne(instance.graph), 2)
    edge_list = collect(edges(instance.graph))
    paths = independent_shortest_paths(instance)
    conflicts = []
    for s1 in 1:length(instance.starts)
        for s2 in (s1 + 1):length(instance.starts)
            for edge in conflict_verification(s1, s2, paths)
                push!(conflicts, edge)
            end
        end
    end
    for edge in 1:ne(instance.graph)
        c = 0
        for item in conflicts
            if edge_list[edge] == item
                c += 1
            end
        end
        features[edge, 1] = c  # conflict-based feature
        features[edge, 2] = degree(instance.graph, dst(edge_list[edge]))  # degree-based feature
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
function linear_regression(edge_features::Array{Int}, regression_weights::Vector{Float64})
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
        instance.graph.weights[src(edge), dst(edge)] = perturbed_θ[i]
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
function training_LR(instance_list, ϵ::Float64, M::Int, α::Float64, num_epochs::Int)
    regression_weights = randn(2) # I'm thinking this should be turning into a matrix: maybe just repeat same weights for each agent? or think about order taken in prioritized planning (though it might make more sense when I use PP as algorithm)
    epoch_list = [x for x in 1:(num_epochs * length(instance_list))]
    loss_list = []
    local y_estimate, fenchel_loss_gradient
    for epoch in 1:num_epochs
        for instance in instance_list
            y_independent_shortest_paths = path_to_binary_matrix(
                instance, independent_shortest_paths(instance)
            )
            features = extract_features(instance)
            weighted_instance = deepcopy(instance)
            y_estimate = zeros(length(instance.starts, ne(instance.graph)))

            θ = linear_regression(features, regression_weights)
            y_m = Vector{Matrix{Int,Int}}(undef, M)
            for m in 1:M
                Z_m = randn(size(θ))
                perturbed_θ = θ + ϵ * Z_m
                weighted_instance = adapt_weights(weighted_instance, perturbed_θ)
                y_m[m] = path_to_binary_matrix(
                    instance, independent_shortest_paths(adapted_instance)
                )
                y_estimate += path_to_binary_matrix(
                    instance, independent_shortest_paths(adapted_instance)
                )
            end
            y_estimate = y_estimate ./ M
            fenchel_loss_gradient = y_independent_shortest_paths - y_estimate
            println(size(fenchel_loss_gradient), size(features), size(regression_weights))
            regression_weights =
                regression_weights - α * (features' * fenchel_loss_gradient)
            println(
                "Epoch $epoch, instance $instance, loss: $fenchel_loss_gradient, regression_weights: $regression_weights",
            )
            push!(loss_list, sum(abs(fenchel_loss_gradient)))
        end
    end
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

    return prioritized_planning(adapted_instance),
    fenchel_loss_gradient,
    path_cost(prioritized_planning(adapted_instance))
end