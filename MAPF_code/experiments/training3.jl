using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs
using MultiAgentPathFinding
using Flux
using InferOpt
using Zygote
using Graphs

file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 11

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, num_agents)

features = MAPF_code.extract_features(instance)
model = Chain(Dense(size(features, 2) => 1), x -> abs.(x))
opt = Flux.Adam(0.01)
opt_state = Flux.setup(opt, model)

oracle =
    θ -> MAPF_code.path_to_binary_vector(
        instance,
        MAPF_code.independent_shortest_paths(
            MAPF_code.adapt_weights(deepcopy(instance), collect(θ))
        ),
    )
layer = PerturbedMultiplicative(oracle; ε=0.1, nb_samples=5);
loss = FenchelYoungLoss(layer);
target_solution = MAPF_code.path_to_binary_vector(
    instance,
    MAPF_code.Solution_to_paths(
        Solution(
            BenchmarkScenario(;
                instance="room-32-32-4",
                scen_type=instance_scen_type,
                type_id=instance_type_id,
                agents=num_agents,
            ),
        ),
        instance,
    ),
)

losses = Float64[]

for epoch in 1:100
    x_input = features'

    grads = Zygote.gradient(model) do m
        θ = m(x_input)
        θ_vec = vec(θ')
        loss(θ_vec, target_solution)
    end

    Flux.update!(opt_state, model, grads[1])

    θ_current = model(x_input)
    θ_vec_current = vec(θ_current')
    current_loss = loss(θ_vec_current, target_solution)
    push!(losses, current_loss)
    if epoch % 10 == 0
        println("Epoch: $epoch")
    end
end

loss_val = loss(vec(model(X')'), y_true)

ne(instance.graph)

# Debug the dimensions
println("Features shape: ", size(features))
println("Model input shape: ", size(features'))

# Test the model output
test_output = model(features')
println("Model output shape: ", size(test_output))
println("Model output vec shape: ", size(vec(test_output')))

# Check target solution
println("Target solution shape: ", size(target_solution))
println("Target solution length: ", length(target_solution))

# Check what the oracle expects
# First, let's see what adapt_weights expects
println("Number of edges in instance: ", ne(instance.graph))

# Test oracle with a simple vector
test_θ = rand(ne(instance.graph)) # This should match the number of edges
println("Test theta length: ", length(test_θ))

try
    test_solution = oracle(test_θ)
    println("Oracle output shape: ", size(test_solution))
    println("Oracle output length: ", length(test_solution))
catch e
    println("Oracle error: ", e)
end