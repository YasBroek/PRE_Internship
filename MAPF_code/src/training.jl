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
    c = 0
    for s1 in 1:length(instance.starts)
        for s2 in 1:length(instance.starts)
            for vertice in conflict_verification(s1, s2, paths)
                push!(conflicts, vertice)
            end
        end
    end
    for edge in 1:ne(instance.graph)
        for vertice in conflicts
            if dst(edge_list[edge]) == vertice
                c += 1
            end
        end
        features[edge, 1] = c # conflict-based feature
        features[edge, 2] = degree(instance.graph, dst(edge_list[edge])) # degree-based feature
    end
    return features
end

function linear_regression(edge_features::Array{Int}, regression_weights::Vector{Float64})
    return edge_features * regression_weights'
end

"""
- Input: z (Instance), y_optimum
- Initialize ε, M (number of perturbations), Z (noise distribution)
- for each epoch do
    - for each instance (x, θ, y_optimum) do
        - θ = GNN_ω(x)
        - ym = []
        - for m in 1:M:
            - Zm ~ Z
            - perturbed_θ <- θ + ε*Zm
            - ym[m] <- parallelized_planning(x, perturbed_θ)
        - end for
        - y_estimate <- sum(ym)/M
        - loss_gradient<- -(y_estimate - y_optimum)     # Fenchel Young loss gradient
        - ω = ω - α * loss_gradient * (dy_estimate/dθ) * (dθ/dω)
    - end for
- end for
"""
function training_LR(
    instance::MAPF_Instance,
    solution_algorithm::Function,
    ϵ::Float64,
    M::Int,
    α::Float64,
    num_epochs::Int,
)
    features = extract_features(instance)
    regression_weights = randn(1, 2)
    for _ in 1:num_epochs
        θ = linear_regression(features, regression_weights)
        y_m = Vector{Float64}(undef, M)
        for m in 1:M
            Z_m = randn(size(θ))
            perturbed_θ = θ + ϵ * Z_m
            y_m[m] = solution_algorithm(x, perturbed_θ) # make it depend on weights
        end
        y_estimate = sum(y_m) / M
        fenchel_loss_gradient = instance.y_optimum - y_estimate
        regression_weights = regression_weights - α * fenchel_loss_gradient * features
    end
end