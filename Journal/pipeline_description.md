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

For training, we use some of the instances provided by the benchmarking. We initially make weight predictions using GNN, then perturb these predictions in m different ways, with copies of Normal distributions multiplied by a temperature constant, using the result to make a Monte Carlo prediction. Finally, we calculate loss using the Fetchel-Young loss function:

L(w) = E_{Z ~ N(0, I)} [ <y_Z, θ + εZ> ] + ε * Ω(y*) - <θ, y*>

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