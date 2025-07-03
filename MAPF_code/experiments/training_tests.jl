using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
ENV["DATADEPS_ALWAYS_ACCEPT"] = true

instance_list = []
base_path = "MAPF_code/input/testing/"

instance_types = filter(isdir, glob("*", base_path))

for instance_folder in instance_types
    type_name = split(instance_folder, '/')[end]

    map_path = joinpath(instance_folder, "$type_name.map")
    if isfile(map_path)
        file_instance = readlines(map_path)
    else
        println("File not found")
        continue
    end

    scen_files = glob("*.scen", instance_folder)

    for scen_path in scen_files
        scen_name = split(scen_path, '/')[end]
        parts = (x -> split(x, '-'))(splitext(scen_name)[1])
        instance_scen_type = parts[end - 1]

        instance_data = readlines(scen_path)

        instance_solution = 12
        instance = MAPF_code.convert_to_my_struct(
            file_instance, instance_data, rand(1:50), instance_solution
        )
        push!(instance_list, instance)
    end
end

MAPF_code.training_LR(instance_list1, 0.1, 10, 0.001, 50)

instance_list1 = [rand(instance_list) for _ in 1:10]

instance_list = []
best_solutions_list = []
instancia = "MAPF_code/input/maze-128-128-10/instance/"
mapa = "MAPF_code/input/maze-128-128-10/instance/maze-128-128-10.map"
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
    push!(instance_list, instance)
    println("c")
    push!(
        best_solutions_list,
        Solution(
            BenchmarkScenario(;
                instance=instance_base_name,
                scen_type=instance_scen_type,
                type_id=instance_type_id,
                agents=1,
            ),
        ),
    )
    println("d")

    for qte_agents in 2:50
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

training_samples = [rand(instance_list) for _ in 1:10]

training_results = MAPF_code.training_LR(training_samples, 0.1, 10, 0.001, 30)

list = list = [Float64[] for _ in 1:10]

for instance in instance_list
    if 31 <= length(instance.starts) <= 40
        cost_path_found = MultiAgentPathFinding.sum_of_costs(
            MAPF_code.calculate_path_v(instance, training_results),
            MAPF(instance.graph, instance.starts, instance.goals),
        )
        diff = (cost_path_found - instance.y_optimum) / instance.y_optimum
        push!(list[length(instance.starts) - 30], diff)
    end
end

boxplot(
    [string(n) for n in 31:40],
    list;
    title="Distance to best found solution",
    ylabel="num_agents",
)

boxplot(
    ["one", "two"],
    [[1, 2, 3, 4, 5], [2, 3, 4, 5, 6, 7, 8, 9]];
    title="Grouped Boxplot",
    xlabel="x",
)

"Open map"
file_instance = readlines(
    open("MAPF_code/input/maze-128-128-10/instance/maze-128-128-10.map")
)

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/maze-128-128-10/instance/maze-128-128-10-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 17

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

path_optimal = MAPF_code.calculate_path(instance, training_results)

MAPF_code.path_cost(instance, path_optimal) # aqui deu 4660

MultiAgentPathFinding.sum_of_costs(
    MAPF_code.calculate_path_v(instance, training_results),
    MAPF(instance.graph, instance.starts, instance.goals),
)

small_scen = BenchmarkScenario(;
    instance="empty-8-8", scen_type="even", type_id=1, agents=32
)
benchmark_solution_best = Solution(small_scen)