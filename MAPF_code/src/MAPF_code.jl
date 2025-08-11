module MAPF_code

using GLMakie
using Graphs
using Cairo: Cairo
using CairoMakie
using Fontconfig: Fontconfig
using Profile
using GraphNeuralNetworks
using Flux
using Distributions
using Random
using LinearAlgebra
using SimpleWeightedGraphs
using InferOpt
using UnicodePlots
using DataStructures
using MetaGraphsNext
using Base.Threads
using SparseArrays
using MultiAgentPathFinding
using ProgressMeter
using Statistics
using CUDA
using Zygote
using ColorSchemes
using MultiAgentPathFinding: NoConflictFreePathError
using StableRNGs

include("structs.jl")
include("TEG_methods.jl")
include("import_data.jl")
include("visualization.jl")
include("find_path.jl")
include("auxiliar_functions.jl")
include("training.jl")

end # module MAPF_code
