using Revise
using MAPF_code
using Graphs

"Open map"
file_instance = readlines(open("input/Berlin_1_256/instance/test.map"))

"Open scenarios"
instance_data = readlines(open("input/Berlin_1_256/instance/test1.scen"))
instance_type_id = 1
instance_scen_type = "even"
num_agents = 2

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(
    file_instance, instance_data, num_agents, instance_solution
)

MAPF_code.visualization(
    file_instance, instance, MAPF_code.independent_shortest_paths(instance)
)

@testset "Testes de index_to_coords" begin
    width = 4  # número de colunas

    @test MAPF_code.index_to_coords(1, width) == (1, 1)
    @test MAPF_code.index_to_coords(2, width) == (2, 1)
    @test MAPF_code.index_to_coords(3, width) == (3, 1)
    @test MAPF_code.index_to_coords(4, width) == (4, 1)

    @test MAPF_code.index_to_coords(5, width) == (1, 2)
    @test MAPF_code.index_to_coords(6, width) == (2, 2)
    @test MAPF_code.index_to_coords(7, width) == (3, 2)
    @test MAPF_code.index_to_coords(8, width) == (4, 2)

    @test MAPF_code.index_to_coords(9, width) == (1, 3)
    @test MAPF_code.index_to_coords(10, width) == (2, 3)

    # Última célula da matriz 3x4
    @test MAPF_code.index_to_coords(12, width) == (4, 3)
end

@info MAPF_code.prioritized_planning(instance)