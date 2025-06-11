"""
Initialize ε, M (number of perturbations), Z (noise distribution)
- Function phi_w:
    - θ = GNN(x)
    - ym = []
    - for m in 1:M:
        - Zm ~ Z
        - perturbed_θ <- θ + ε*Zm
        - ym[m] <- parallelized_planning(x, perturbed_θ)
    - end
    - y_estimate <- sum(ym)/M
    - F_ε <- (1/M) * sum([<y_m, θ + ε * Zm> for y_m in ym])
    - loss <- F_ε + ε * Ω(y_optimum) - <θ, y_optimum>
    - return loss, θ
- end
"""

ε = 0.1
M = 10

function ϕ_w(x::MAPF_Instance)
    θ = [rand() for e in collect(edges(x.graph))]
    y_m = Vector{Float64}(undef,M)
    for m in 1:M
        Z_m = 
        perturbed_θ = θ + ϵ*Z_m
        y_m[m] = parallelized_planning(x, perturbed_θ)

    end
    y_estimate = sum(y_m)/M
end