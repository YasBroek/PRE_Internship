using Revise
using MAPF_code
using Graphs
using SimpleWeightedGraphs
using MultiAgentPathFinding
using ProgressMeter

"Open map"
file_instance = readlines(
    open("MAPF_code/input/random-32-32-20/instance/random-32-32-20.map")
)

"Open scenarios"
instance_data = readlines(
    open("MAPF_code/input/random-32-32-20/instance/random-32-32-20-even-1.scen")
)
instance_type_id = 1
instance_scen_type = "even"
num_agents = 32

instance_solution = 12
instance = MAPF_code.convert_to_my_struct(file_instance, instance_data, num_agents)

mapf = MAPF(instance.graph, instance.starts, instance.goals)

PP = MAPF_code.prioritized_planning_v2(instance)
@info PP[2]
@info instance.goals[2]

list = [[s] for s in instance.starts]

for path in 1:length(PP)
    for edge in PP[path]
        push!(list[path], dst(edge))
    end
end

list
PP

is_feasible(Solution(list), mapf; verbose=true)

MAPF_code.visualization(file_instance, instance, PP)

small_scen = BenchmarkScenario(;
    instance="random-32-32-20", scen_type="even", type_id=1, agents=num_agents
)

plot_mapf(small_scen, Solution(list); video_path=joinpath(@__DIR__, "solution.mp4"))

sum_of_costs(cooperative_astar(mapf, collect(1:num_agents)), mapf)

MAPF_code.path_cost(
    instance,
    MAPF_code.Solution_to_paths(cooperative_astar(mapf, collect(1:num_agents)), instance),
)