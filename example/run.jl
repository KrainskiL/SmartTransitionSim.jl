using SmartTransitionSim

#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/SmartTransitionSim.jl/example";
RoadSet = 5
map_data = get_map_data(datapath, mapfile, use_cache=false; road_levels = Set(1:RoadSet));

#Defining starting and ending area
Start = [Rect((39.50,-119.70),(39.55,-119.74))]
End = [Rect((39.50,-119.80),(39.55,-119.76))]

#Dictionary for memorizing used k-shortest paths
KDict = Dict{Tuple{Int,Int},Array{Vector{Int}}}()

#Declaring simulation input parameters
density_factor = 5.0
smart_perc = 0.5
agents_num = 1000
update_period = 150
T = 0.1
k = 3

#Generating agents
Agents = generate_agents(map_data, agents_num, Start, End, smart_perc, k, T , KDict)
#Running base simulation - no V2I system
BaseOutput = simulation_run(:base, map_data, Agents)
SmartOutput = simulation_run(:smart, map_data, Agents, KDict, update_period, T, k, density_factor)

simulation_statistics = gather_statistics(getfield.(Agents,:smart),
                                    BaseOutput.TravelTimes,
                                    SmartOutput.TravelTimes)
#Gathered simulation statistics - percentage time reduction
println(simulation_statistics)
