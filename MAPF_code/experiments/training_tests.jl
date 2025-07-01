using Revise
using MAPF_code
using Glob

instance_list = []
base_path = "MAPF_code/input/testing/"

tipos_instancias = filter(isdir, glob("*", base_path))

for pasta_instancia in tipos_instancias
    println("Processando: $pasta_instancia")

    tipo_nome = split(pasta_instancia, '/')[end]

    # Caminho do arquivo .map
    caminho_map = joinpath(pasta_instancia, "$tipo_nome.map")
    if isfile(caminho_map)
        file_instance = readlines(caminho_map)
        println("Leu mapa com $(length(file_instance)) linhas.")
    else
        println("Arquivo .map não encontrado: $caminho_map")
        continue
    end

    # Lê todos os arquivos .scen da pasta
    arquivos_scen = glob("*.scen", pasta_instancia)

    for caminho_scen in arquivos_scen
        # Extrai tipo (ex: even, odd, etc.) do nome do arquivo
        nome_scen = split(caminho_scen, '/')[end]
        partes = (x -> split(x, '-'))(splitext(nome_scen)[1])
        instance_scen_type = partes[end - 1]  # geralmente "even", "odd", etc.

        instance_data = readlines(caminho_scen)
        println("Leu cenário: $nome_scen com $(length(instance_data)) linhas.")

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

# Lê todos os arquivos .scen da pasta
arquivos_scen = glob("*.scen", instancia)

for caminho_scen in arquivos_scen
    # Extrai tipo (ex: even, odd, etc.) do nome do arquivo
    nome_scen = split(caminho_scen, '/')[end]
    partes = (x -> split(x, '-'))(splitext(nome_scen)[1])
    instance_scen_type = partes[end - 1]  # geralmente "even", "odd", etc.

    instance_data = readlines(caminho_scen)
    println("Leu cenário: $nome_scen com $(length(instance_data)) linhas.")

    instance_solution = 12
    instance = MAPF_code.convert_to_my_struct(
        file_instance, instance_data, rand(1:50), instance_solution
    )
    push!(instance_list, instance)
end

@profview for _ in 1:5
    MAPF_code.training_LR(instance_list1, 0.1, 10, 1e-5, 5)
end

instance_list1 = [rand(instance_list) for _ in 1:10]

training_results = MAPF_code.training_LR(instance_list1, 0.1, 10, 0.001, 30)

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
