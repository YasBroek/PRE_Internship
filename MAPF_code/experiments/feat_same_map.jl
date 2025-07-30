using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
using JLD2
using InferOpt
using Flux: softplus
using Graphs
using Random
ENV["DATADEPS_ALWAYS_ACCEPT"] = true

instance_list = []
best_solutions_list = []

instancia = "MAPF_code/input/room-32-32-4/training/"

mapa = "MAPF_code/input/room-32-32-4/training/room-32-32-4.map"

file_instance = readlines(mapa)

arquivos_scen = glob("*.scen", instancia)

nome_scen = split(arquivos_scen[1], '/')[end]
nome_sem_ext = splitext(nome_scen)[1]
partes = split(nome_sem_ext, '-')
instance_base_name = String(join(partes[1:(end - 2)], '-'))
instance_scen_type = String(partes[end - 1])
instance_type_id = parse(Int, String(partes[end]))

instance_data = readlines(arquivos_scen[1])

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, rand(1:50))
i = 0
l = length(arquivos_scen)

for caminho_scen in arquivos_scen
    i += 1
    println("$i / $l")
    nums = []
    for _ in 1:8
        nome_scen = split(caminho_scen, '/')[end]
        nome_sem_ext = splitext(nome_scen)[1]
        partes = split(nome_sem_ext, '-')
        instance_base_name = String(join(partes[1:(end - 2)], '-'))
        instance_scen_type = String(partes[end - 1])
        instance_type_id = parse(Int, String(partes[end]))

        instance_data = readlines(caminho_scen)
        println("a")
        instance = MAPF_code.change_scenarios(instance_data, instance, 1)
        println("b")

        qte_agents = rand(1:60)
        while qte_agents in nums
            qte_agents = rand(1:60)
        end
        push!(nums, qte_agents)

        instance = MAPF_code.increase_agent_quantity(instance_data, instance, qte_agents)
        push!(instance_list, instance)
        push!(
            best_solutions_list,
            Solution(
                BenchmarkScenario(;
                    instance=instance_base_name,
                    scen_type=instance_scen_type,
                    type_id=instance_type_id,
                    agents=qte_agents,
                ),
            ),
        )
        println("agents: $qte_agents")
    end
end

@save "instances_100_even_room" instance_list
@save "solutions_100_even_room" best_solutions_list

model, losses = MAPF_code.training(instance_list, best_solutions_list, 0.001, 10, 0.1, 80)

function compute_average_path_cost_gdalle(num_agents::Int, sampled_numbers)
    instance = "room-32-32-4"
    scen_type = "random"
    impact = 0.0
    valid_count = 0  # To count valid scenarios

    for type_id in sampled_numbers
        cost_original = 0.0
        cost_new = 0.0
        scen = BenchmarkScenario(; instance, scen_type, type_id, agents=num_agents)
        bench_mapf = MAPF(scen; allow_diagonal_moves=false)

        try
            cost_original = sum_of_costs(
                cooperative_astar(bench_mapf, collect(1:num_agents)), bench_mapf
            )
            new_instance = MAPF_code.MAPF_Instance(
                32, 32, bench_mapf.graph, bench_mapf.departures, bench_mapf.arrivals, [], []
            )
            features = MAPF_code.extract_features(new_instance)
            x_input = features'
            weights = model(x_input)
            θ_vec = vec(weights')

            weighted_instance = MAPF_code.adapt_weights(
                deepcopy(new_instance), collect(θ_vec)
            )
            mapf = MAPF(weighted_instance.graph, new_instance.starts, new_instance.goals)
            cost_new = sum_of_costs(
                cooperative_astar(mapf, collect(1:num_agents)), bench_mapf
            )

            impact += cost_new / cost_original
            valid_count += 1  # Increment valid count if no error occurs
        catch e
            # Optionally log the error or print a message
            println("Error encountered for type_id $type_id: $e")
            continue  # Skip to the next iteration
        end
    end

    # Calculate the average cost only if there are valid scenarios
    final_impact = valid_count > 0 ? impact / valid_count : 0.0
    return final_impact
end

# Define the number of agents to test
agent_counts = [10, 25, 60]

list = randperm(25)[1:5]

# Compute and print the average path costs for each number of agents
for agents in agent_counts
    average_cost = compute_average_path_cost_gdalle(agents, list)
    println("Average path cost for $agents agents: $average_cost")
end