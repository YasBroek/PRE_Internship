using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots

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
instancia = "MAPF_code/input/maze-128-128-10/instance/"
mapa = "MAPF_code/input/maze-128-128-10/instance/maze-128-128-10.map"
file_instance = readlines(mapa)

arquivos_scen = glob("*.scen", instancia)

for caminho_scen in arquivos_scen
    nome_scen = split(caminho_scen, '/')[end]
    partes = (x -> split(x, '-'))(splitext(nome_scen)[1])
    instance_scen_type = partes[end - 1]
    instance_type_id = partes[end]

    instance_data = readlines(caminho_scen)

    for qte_agents in 1:50
        solutions = readlines(
            open("MAPF_code/input/maze-128-128-10/solution/maze-128-128-10.csv")
        )

        solutions_header = split(solutions[1], ",")
        solution_costs = Dict{Tuple{String,Int,Int},Float64}()
        for line in solutions[2:end]
            values = split(line, ",")
            scen_type = strip(values[1], '"')
            type_id = parse(Int, strip(values[2], '"'))
            agents = parse(Int, strip(values[3], '"'))
            solution_cost = parse(Float64, strip(values[6], '"'))

            key = (scen_type, type_id, agents)
            solution_costs[key] = solution_cost
        end

        instance_solution = solution_costs[(
            instance_scen_type, parse(Int, instance_type_id), qte_agents
        )]

        instance = MAPF_code.convert_to_my_struct(
            file_instance, instance_data, qte_agents, instance_solution
        )
        push!(instance_list, instance)
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