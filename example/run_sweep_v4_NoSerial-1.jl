const i10 = parse(Int, ARGS[1])

using SmartTransitionSim
using StatsBase
using Random

#Creating MapData object
const mapfile = "SanFranciscoFiltered.osm"
const datapath = "./example/";
const RoadSet = 5
const map_data = get_map_data(datapath, mapfile, use_cache=false; road_levels = Set(1:RoadSet));

#Dictionary for memorizing used k-shortest paths
const KDict = Dict{Tuple{Int,Int},Array{Vector{Int}}}()

#Create parameters grid
const density_factor = 5.0
const Params = (reps = 1:3, α = 0.05:0.05:1.0, N = 3000:500:7500, U = 50:50:300, T = [0.1,1.0,10.0], k = [1,2,3,4,5], AvgDelay = 0.0)
const ParametersGridProd = collect(Base.Iterators.product(Params.reps,Params.α, Params.N, Params.U, Params.T, Params.k, Params.AvgDelay))
const ParametersGrid = [ParametersGridProd[i] for i in 1:length(ParametersGridProd)]

#Defining starting and ending area
const Start = [Rect((37.795,-122.42),(37.811,-122.3942))]
const End = [Rect((37.7805,-122.485),(37.79,-122.45))]

#Create Agents Pool
T = 0.5
kmode = mode([ParametersGrid[i][6] for i in (i10*40-39):(i10*40)])
Delay = ParametersGrid[1][7]

Random.seed!(1);

Agents1 = generate_agents(map_data, 5000, Start, End, 0.0, kmode, T, Delay, KDict)
Agents2 = generate_agents(map_data, 5000, Start, End, 0.0, kmode, T, Delay, KDict)
const Agents = [Agents1; Agents2]
#Output file header
println("ID,rep,α,N,U,T,k,AvgDelay,runtime,overall_time_red,smart_time_red,regular_time_red,avg_time_base,avg_time_all_V2I,avg_time_smart_V2I,avg_time_regular_V2I")

for i in (i10*40-39):(i10*40)
    Random.seed!(1000);
    run_parameter_analysis(i,ParametersGrid,Agents,density_factor,KDict,map_data)
    flush(stdout)
end
