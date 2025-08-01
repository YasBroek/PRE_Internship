using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
using JLD2
using InferOpt
using Flux: softplus
using Graphs
ENV["DATADEPS_ALWAYS_ACCEPT"] = true

instance_list = []
best_solutions_list = []
instancia = "MAPF_code/input/Berlin_1_256/instance/"

mapa = "MAPF_code/input/Berlin_1_256/instance/Berlin_1_256.map"

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

@save "room-32-32-4-random" instance_list

model, losses = MAPF_code.training_weights(
    instance_list, best_solutions_list, 0.001, 10, 0.1, 240, instance
)
@info model
using CairoMakie
using Makie: get_colormap

MAPF_code.visualize_edge_weights(file_instance, instance_list[1], model)
MAPF_code.visualization(
    file_instance, instance_list[1], MAPF_code.prioritized_planning_v2(instance_list[1])
)
"Open map"
file_instance = readlines(open("MAPF_code/input/room-32-32-4/instance/room-32-32-4.map"))

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/room-32-32-4/instance/room-32-32-4-even-1.scen")
)

instance_type_id = 1
instance_scen_type = "even"
num_agents = 20

instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, num_agents)

features = MAPF_code.extract_features(instance)
length(features)
x_input = features'
weights = model(x_input)
θ_vec = vec(weights')

mapf = MAPF(instance.graph, instance.starts, instance.goals)
mapf_w = MAPF(
    MAPF_code.adapt_weights(instance, model).graph, instance.starts, instance.goals
)

PP_cost = sum_of_costs(cooperative_astar(mapf, collect(1:length(instance.starts))), mapf)

trained_cost = sum_of_costs(
    cooperative_astar(mapf_w, collect(1:length(instance.starts))), mapf
)

cost1 = MAPF_code.path_cost(instance, MAPF_code.prioritized_planning_v2(instance))

weighted_instance = MAPF_code.adapt_weights(deepcopy(instance), model)
cost2 = MAPF_code.path_cost(instance, MAPF_code.prioritized_planning_v2(weighted_instance))

@info MAPF_code.prioritized_planning_v2(weighted_instance)

weighted_instance = MAPF_code.adapt_weights(deepcopy(instance), collect(θ_vec))
@info ne(weighted_instance.graph)
mapf = MAPF(weighted_instance.graph, instance.starts, instance.goals)
a = 0
for w in weighted_instance.graph.weights
    if w > 0
        a += 1
    end
end
a

cost = cooperative_astar(mapf, collect(1:length(instance.starts)))