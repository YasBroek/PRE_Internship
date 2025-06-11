# Development of formal description for pipeline

## Pipeline Overview

Given a Multi-Agent Path Finding (MAPF) instance `x`, we aim to learn a function `ϕ_w` that outputs edge weights `θ`, which are then used in an algorithm (prioritized planning) to produce a solution `y`. To enable end-to-end training, we apply a smoothing technique based on stochastic perturbations, enabling a differentiable surrogate objective.

---

## Statistical Model

The model `ϕ_w` is a GNN that maps MAPF instances x to edge weights θ. Perturbations `Z_m` with a normal distribution are applied to the learned parameters, scaled by a temperature parameter `ε`, to allow gradient estimation via smoothing.
- Input: Instance (graph representing MAPF environment, along with agents' starts and goals and global optimal value)
- Output: Weight vector `θ = ϕ_w(x)`.

--- 

## Optimization Problem

Our objective is to

maximize    `∑_{a ∈ A}(∑_{e ∈ E} θ_e * (y^a)_e)`

given a set of agents A = (`a_1`, `a_2`, ..., `a_n`) and a graph G = (V, E), where `θ_e` represent the predicted weight for edge e, and `(y^a)_e` are binary variables, equal to 1 if agent a passes throught edge e and 0 otherwise. 
As for the constraints, we have to make sure of the flow conservation for the path of each agent, and that there aren't any conflicts between agents at each time step.


---

## Training Procedure

Input: x (Instance), y_optimum
Initialize epsilon, M (number of perturbations), Z (noise distribution), n (learning rate)
Function phi_w:
    theta = GNN(x)
    ym = []
    for m in 1:M:
        Zm ~ Z
        perturbed_theta <- theta + epsilon*Zm
        ym[m] <- parallelized_planning(x, perturbed_theta)
    end
    y_estimate <- sum(ym)/M
    F_epsilon <- (1/M) * sum([<y_m, θ + ε * Zm> for y_m in ym])
    loss <- F_epsilon + epsilon * Omega(y_optimum) - <theta, y_optimum>
    return loss, y_estimate
end