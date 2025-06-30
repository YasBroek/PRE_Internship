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

MAPF_code.training_LR(instance_list1, 0.1, 10, 1e-5, 100)

instance_list1 = [rand(instance_list) for _ in 1:5]