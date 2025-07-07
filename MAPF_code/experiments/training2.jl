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

training_results = MAPF_code.training_LR(
    instance_list, best_solutions_list, 0.01, 10, 0.001, 500
)

"Open map"
file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 10

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, num_agents)

sum_of_costs(
    cooperative_astar(MAPF(instance.graph, instance.starts, instance.goals)),
    MAPF(instance.graph, instance.starts, instance.goals),
)
adapted_instance = MAPF_code.adapt_weights(
    instance,
    MAPF_code.generalized_linear_model(
        MAPF_code.extract_features(instance), training_results
    ),
)
sum_of_costs(
    cooperative_astar(MAPF(adapted_instance.graph, instance.starts, instance.goals)),
    MAPF(instance.graph, instance.starts, instance.goals),
)

cooperative_astar(MAPF(adapted_instance.graph, instance.starts, instance.goals))

PP = MAPF_code.path_cost(instance, MAPF_code.prioritized_planning_v2(instance))

adapted_instance = MAPF_code.adapt_weights(
    instance,
    MAPF_code.generalized_linear_model(
        MAPF_code.extract_features(instance), training_results
    ),
)

MAPF_code.path_cost(instance, MAPF_code.prioritized_planning_v2(adapted_instance))