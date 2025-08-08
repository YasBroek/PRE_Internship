using Revise
using MAPF_code
using Glob
using MultiAgentPathFinding
using UnicodePlots
using JLD2
using InferOpt
using Flux: softplus
using Graphs
using Random
ENV["DATADEPS_ALWAYS_ACCEPT"] = true

instance = "empty-32-32"
scen_type = "random"
type_id = 5
agents = 20
scen = BenchmarkScenario(; instance, scen_type, type_id, agents)
bench_mapf = MAPF(scen; allow_diagonal_moves=true)
benchmark_solution_best = Solution(scen)