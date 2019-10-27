module SmartTransitionSim

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays
using Base.Iterators
using LightGraphs
using DataFrames
using DataStructures
using Random

export get_map_data

#types
export Rect, Agent

#functions
#generate_agents.jl
export generate_agents, pick_random_node, get_nodes_set
#rerouting.jl
export k_shortest_path_rerouting!, yen_a_star
#simulations.jl
export simulation_run, gather_statistics, run_parameter_analysis
#traffic_model.jl
export get_max_densities, traffic_constants, init_traffic_variables, next_edge
export update_weights!, update_event_agent!, update_agents_position!, update_weights_and_events!

#files
include("types.jl")
include("generate_agents.jl")
include("rerouting.jl")
include("simulations.jl")
include("traffic_model.jl")

end
