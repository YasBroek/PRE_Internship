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

ϵ = 0.1
M = 10
epochs = 5
ω = # GNN model
# linear regression
    α = 0.1

for epoch in 1:epochs
    θ = ω(instance)
    y_m = Vector{Float64}(undef, M)
    for m in 1:M
        Z_m = randn(size(θ))
        perturbed_θ = θ + ϵ * Z_m
        y_m[m] = prioritized_planning(x, perturbed_θ)
    end
    y_estimate = sum(y_m) / M
    fenchel_loss_gradient = instance.y_optimum - y_estimate
    ω = ω - α * fenchel_loss_gradient * (dθ / dω)
end

# use independent shortest paths as feature