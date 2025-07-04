using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
using JLD2
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

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, 1)
i = 0
l = length(arquivos_scen)

caminho_scen = arquivos_scen[1]
i += 1
println("$i / $l")
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

qte_agents = rand(1:50)

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

for caminho_scen in arquivos_scen
    i += 1
    println("$i / $l")
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

    qte_agents = rand(1:50)

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

@load "training_room-32-32-4_instances.jld2" instance_list best_solutions_list

training_results = MAPF_code.training_LR(
    instance_list, best_solutions_list, 0.1, 10, 0.001, 30
)

@profview for _ in 1:3
    MAPF_code.training_LR(instance_list, best_solutions_list, 0.1, 10, 0.001, 1)
end

@profview MAPF_code.training_LR(instance_list, best_solutions_list, 0.1, 10, 0.001, 3)

θ = randn(9)
perturbed_θ = θ .+ 0.1 * randn(9)
graph = MAPF_code.adapt_weights(instance_list[1], perturbed_θ).graph
MAP = MAPF(graph, instance_list[1].starts, instance_list[1].goals)
path = cooperative_astar(MAP)

y_best_found_solution = MAPF_code.path_to_binary_vector(
    instance_list[1], MAPF_code.Solution_to_paths(best_solutions_list[1], instance_list[1])
)

fenchel_young_loss(
    instance_list[1],
    extract_features(instance_list[1]),
    10,
    randn(9),
    y_best_found_solution,
    Z_m,
    0.1,
)