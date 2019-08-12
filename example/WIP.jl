#Creating MapData object
mapfile = "SanFranciscoFiltered.osm"
datapath = "C:/SmartTransitionSim.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));
#Defining starting and ending area
Start = [Rect((37.795,-122.42),(37.811,-122.3942))]
End = [Rect((37.7805,-122.485),(37.79,-122.45))]

α = 0.6
N = 1000
density_factor = 5.0
updt_period = 200
T = 0.1
k = 3

#Generating agents
Agents = generate_agents(map_data, N, Start, End, α)[1]

@time ITSOutput = simulation_run("smart", map_data,
                                                Agents,
                                                updt_period,
                                                T,
                                                k,
                                                density_factor,
                                                debug_level=2)

# include("C:/RSUOptimizationVis.jl/src/RSUOptimizationVis.jl")
# RSUOptimizationVis.visualize_bounds(map_data,Start,End,"TEST.html")
# RSUOptimizationVis.visualize_RSUs_and_failures(map_data, Start, End, Agents, [ITSOutput.FailedUpdates[1]],RSUs,range,"debug.html")
