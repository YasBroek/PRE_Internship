using Revise
using MAPF_code
using MultiAgentPathFinding
using Graphs
using CairoMakie
using Statistics
using MultiAgentPathFinding: NoConflictFreePathError
using DataFrames
using CSV

function read_benchmark_solution(scen::BenchmarkScenario)
    (; instance, scen_type, type_id, agents) = scen
    sol_path = joinpath(
        "MAPF_code",
        "input",
        instance,
        "solution",
        string(instance, "-", scen_type, "-", type_id, "-", agents, "-sol.csv"),
    )
    sol_df = DataFrame(CSV.File(sol_path))
    if size(sol_df, 1) == 0
        throw(
            MissingSolutionError(
                "Scenario $scen_type-$type_id does not exist for instance $instance"
            ),
        )
    end
    agents = if isnothing(agents)
        maximum(sol_df[!, :agents])
    else
        agents
    end
    right_agents = sol_df[!, :agents] .== agents
    sol_df = sol_df[right_agents, :]
    if size(sol_df, 1) == 0
        throw(
            MissingSolutionError(
                "Scenario $scen_type-$type_id for instance $instance does not have a best known solution with $agents agents",
            ),
        )
    end
    sol = only(eachrow(sol_df))
    plan = sol[:path]
    if ismissing(plan)
        throw(
            MissingSolutionError(
                "Scenario $scen_type-$type_id for instance $instance does not have a best known solution with $agents agents",
            ),
        )
    end
    paths_string_list = split(plan, "\n")

    agent_list = MultiAgentPathFinding.read_benchmark_scenario(scen)

    paths_coord_list = map(1:agents, paths_string_list) do a, path_string
        start = (agent_list[a].start_i, agent_list[a].start_j)
        goal = (agent_list[a].goal_i, agent_list[a].goal_j)
        location = start
        path_coord = [location]
        for c in path_string
            if c == 'u'  # up means y+1 means i+1
                location = location .+ (1, 0)
            elseif c == 'd'  # down means y-1 means i-1
                location = location .+ (-1, 0)
            elseif c == 'l'  # left means x-1 means j-1
                location = location .+ (0, -1)
            elseif c == 'r'  # right means x+1 means j+1
                location = location .+ (0, 1)
            elseif c == 'w'  # wait means nothing changes
                location = location .+ (0, 0)
            end
            push!(path_coord, location)
        end
        @assert path_coord[begin] == start
        @assert path_coord[end] == goal
        return path_coord
    end

    return (;
        lower_cost=sol[:lower_cost],
        solution_cost=sol[:solution_cost],
        paths_coord_list=paths_coord_list,
    )
end

function Solution(scen::BenchmarkScenario; check::Bool=false)
    grid = read_benchmark_map(scen.instance)
    _, coord_to_vertex, _ = parse_benchmark_map(grid; allow_diagonal_moves=false)
    lower_cost, solution_cost, paths_coord_list = read_benchmark_solution(scen)
    solution = MultiAgentPathFinding.Solution(
        map(path_coord -> getindex.(Ref(coord_to_vertex), path_coord), paths_coord_list)
    )
    if check
        mapf = MAPF(scen)
        @assert is_feasible(solution, mapf; verbose=true)
        @assert lower_cost <= sum_of_costs(solution, mapf) == solution_cost
    end
    return solution
end

training_set = []
training_set_solutions = []
training_names = []
test_set = []
test_names = []
validation_set = []
validation_names = []
validation_solutions = []

i = 0
agents_list6 = []
instance = "room-64-64-8"
scen_type = "even"
for type_id in 1:10
    agents = 100
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
    bench_mapf = MAPF(scen; allow_diagonal_moves=false)
    got_it = false
    while !got_it
        try
            cooperative_astar(bench_mapf, collect(1:agents))
            got_it = true
        catch e
            agents = agents - 1
            scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
            bench_mapf = MAPF(scen; allow_diagonal_moves=false)
        end
    end
    push!(agents_list5, agents)
    i += 1
    println(i)
end
@info agents_list5

instance_list = [
    "empty-8-8",
    "empty-32-32",
    "maze-32-32-2",
    "maze-32-32-4",
    "random-32-32-20",
    "random-64-64-10",
    "room-32-32-4",
    "room-64-64-8",
]
scen_type = "even"
agents_list = [
    [32, 32, 22, 28, 24, 22, 20, 32, 24, 16],
    [100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
    [19, 9, 43, 62, 18, 31, 43, 27, 16, 32],
    [57, 38, 52, 31, 38, 8, 27, 75, 33, 100],
    [100, 100, 100, 78, 100, 90, 100, 77, 60, 100],
    [100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
    [36, 61, 88, 47, 77, 43, 40, 61, 76, 27],
    [94, 30, 52, 3, 50, 19, 66, 51, 100, 4],
]
for (index, instance) in enumerate(instance_list)
    for type_id in 1:3
        try
            scen = BenchmarkScenario(;
                instance, scen_type, type_id, agents=agents_list[index][type_id]
            )
            @info scen

            # Try to create the MAPF problem and solution
            mapf_problem = MAPF(scen; allow_diagonal_moves=false)
            good_sol = Solution(scen)

            # If we get here, everything worked - add to training set
            push!(training_set, mapf_problem)
            push!(training_set_solutions, good_sol)
            push!(training_names, instance)

        catch e
            if e isa AssertionError || e isa MissingSolutionError
                @warn "Skipping scenario $(instance)-$(scen_type)-$(type_id): $(e)"
                continue
            else
                # Re-throw unexpected errors
                rethrow(e)
            end
        end
    end
end

validation_names = [
    "empty-8-8",
    "empty-32-32",
    "empty-48-48",
    "maze-32-32-2",
    "maze-32-32-4",
    "maze-128-128-10",
    "random-32-32-20",
    "random-64-64-10",
    "random-64-64-20",
    "room-32-32-4",
    "room-64-64-8",
    "room-64-64-16",
]
scen_type = "random"
type_id = 1
agents_list_val = [2, 34, 48, 5, 5, 47, 27, 54, 22, 33, 85, 18]
for (index, instance) in enumerate(validation_names)
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents=agents_list_val[index])
    @info scen
    good_sol = Solution(scen)
    push!(validation_set, MAPF(scen; allow_diagonal_moves=false))
    push!(validation_solutions, good_sol)
end

list_e48 = []
instance = "empty-48-48"
type_id = 25
scen_type = "random"
@showprogress for agents in 1:100
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
    test_bench_mapf = MAPF(scen; allow_diagonal_moves=false)
    path_cost_original = sum_of_costs(cooperative_astar(test_bench_mapf), test_bench_mapf)
    features = MAPF_code.extract_features_gdalle(test_bench_mapf, instance)
    features_batch = features'

    weights_batch = model(features_batch)
    θ_vec_current = vec(weights_batch)

    weighted_instance = MAPF_code.adapt_weights(test_bench_mapf, -θ_vec_current)

    path_cost = sum_of_costs(cooperative_astar(weighted_instance), test_bench_mapf)
    push!(list_e48, path_cost / path_cost_original)
end

fig1 = Figure(; resolution=(800, 500))

ax1 = Axis(
    fig1[1, 1];
    xlabel="Number of agents",
    ylabel="Cost Ratio",
    title="Cost Ratio vs Number of agents",
)

lines!(ax1, 1:100, list_e48; color=:blue, linewidth=2)
display(fig1)

test_names = [
    "empty-8-8",
    "empty-32-32",
    "empty-48-48",
    "maze-32-32-2",
    "maze-32-32-4",
    "maze-128-128-10",
    "random-32-32-20",
    "random-64-64-10",
    "random-64-64-20",
    "room-32-32-4",
    "room-64-64-8",
    "room-64-64-16",
]
test_scen_type = "random"
test_type_id = 25
test_agents = 20
for test_instance in test_names
    test_scen = BenchmarkScenario(;
        instance=test_instance,
        scen_type=test_scen_type,
        type_id=test_type_id,
        agents=test_agents,
    )
    test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)
    push!(test_set, test_bench_mapf)
end
using ProgressMeter
features_list = []
@showprogress "Extracting features" for (index, instance) in enumerate(training_set)
    push!(features_list, MAPF_code.extract_features_gdalle(instance, training_names[index]))
end

my_features_list = deepcopy(features_list)
my_training_set = deepcopy(training_set)
my_training_names = deepcopy(training_names)
my_training_set_solutions = deepcopy(training_set_solutions)

model, losses, val_avg_10 = MAPF_code.training_gdalle(
    my_training_set,
    my_training_names,
    my_training_set_solutions,
    my_features_list,
    1.0,
    2,
    0.0001,
    200,
    validation_set,
    validation_names,
)

test_names = ["room-32-32-4", "room-64-64-8", "room-64-64-16"]
test_scen_type = "random"
test_type_id = 20
test_agents = [30, 44, 43]
println(test_agents)
for (index, instance) in enumerate(test_names)
    test_scen = BenchmarkScenario(;
        instance=instance,
        scen_type=test_scen_type,
        type_id=test_type_id,
        agents=test_agents[index],
    )
    test_bench_mapf = MAPF(test_scen; allow_diagonal_moves=false)
    path_cost_original = sum_of_costs(cooperative_astar(test_bench_mapf), test_bench_mapf)
    features = MAPF_code.extract_features_gdalle(test_bench_mapf, test_names[index])
    features_batch = features'

    weights_batch = model(features_batch)
    θ_vec_current = vec(weights_batch)

    weighted_instance = MAPF_code.adapt_weights(test_bench_mapf, -θ_vec_current)

    path_cost = sum_of_costs(cooperative_astar(weighted_instance), test_bench_mapf)

    println(
        "instance: $instance, number of agents: $(test_agents[index]), cost ratio: $(path_cost/path_cost_original)",
    )
end

instance_list_room = ["room-32-32-4", "room-64-64-8"]
scen_type_room = "even"
agents_list_room = [
    [36, 61, 88, 47, 77, 43, 40, 61, 76, 27], [94, 30, 52, 3, 50, 19, 66, 51, 100, 4]
]
for (index, instance) in enumerate(instance_list_room)
    for type_id in 1:3
        try
            scen = BenchmarkScenario(;
                instance,
                scen_type=scen_type_room,
                type_id,
                agents=agents_list_room[index][type_id],
            )
            @info scen

            # Try to create the MAPF problem and solution
            mapf_problem = MAPF(scen; allow_diagonal_moves=false)
            good_sol = Solution(scen)

            # If we get here, everything worked - add to training set
            push!(training_set, mapf_problem)
            push!(training_set_solutions, good_sol)
            push!(training_names, instance)

        catch e
            if e isa AssertionError || e isa MissingSolutionError
                @warn "Skipping scenario $(instance)-$(scen_type)-$(type_id): $(e)"
                continue
            else
                # Re-throw unexpected errors
                rethrow(e)
            end
        end
    end
end

validation_names = ["room-32-32-4", "room-64-64-8", "room-64-64-16"]
scen_type = "random"
type_id = 1
agents_list_val = [33, 85, 18]
for (index, instance) in enumerate(validation_names)
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents=agents_list_val[index])
    @info scen
    good_sol = Solution(scen)
    push!(validation_set, MAPF(scen; allow_diagonal_moves=false))
    push!(validation_solutions, good_sol)
end

features_list = []
@showprogress "Extracting features" for (index, instance) in enumerate(training_set)
    push!(features_list, MAPF_code.extract_features_gdalle(instance, training_names[index]))
end

model_room, losses_room, val_avg_10 = MAPF_code.training_gdalle(
    training_set,
    training_names,
    training_set_solutions,
    features_list,
    1.0,
    2,
    0.0001,
    200,
    validation_set,
    validation_names,
)

list_r_r6416 = []
instance = "room-64-64-16"
type_id = 25
scen_type = "random"
@showprogress for agents in 1:100
    scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
    test_bench_mapf = MAPF(scen; allow_diagonal_moves=false)
    path_cost_original = sum_of_costs(cooperative_astar(test_bench_mapf), test_bench_mapf)
    features = MAPF_code.extract_features_gdalle(test_bench_mapf, instance)
    features_batch = features'

    weights_batch = model_room(features_batch)
    θ_vec_current = vec(weights_batch)

    weighted_instance = MAPF_code.adapt_weights(test_bench_mapf, -θ_vec_current)

    path_cost = sum_of_costs(cooperative_astar(weighted_instance), test_bench_mapf)
    push!(list_r_r6416, path_cost / path_cost_original)
end

fig1 = Figure(; resolution=(800, 500))

ax1 = Axis(
    fig1[1, 1];
    xlabel="Number of agents",
    ylabel="Cost Ratio",
    title="Cost Ratio vs Number of agents",
)

lines!(ax1, 1:100, list_r_r6416; color=:blue, linewidth=2)
display(fig1)