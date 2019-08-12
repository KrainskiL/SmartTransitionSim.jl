using OpenStreetMapX
using SmartTransitionSim
using CSV
using DataFrames

#Creating MapData object
mapfile = "reno_east3.osm";
datapath = "C:/SmartTransitionSim.jl/example";
RoadSet = 5;
map_data = OpenStreetMapX.get_map_data(datapath, mapfile, use_cache = false; road_levels = Set(1:RoadSet));

#Defining starting and ending area
Start = [Rect((39.50,-119.70),(39.55,-119.74))]
End = [Rect((39.50,-119.80),(39.55,-119.76))]

#Input parameters
α = 0.5
N = 800
density_factor = 5.0
updt_period = 200
T = 0.1
k = 3

"""
Parameters analysis
"""
ResultFrame = DataFrame(Map = String[],
              RoadSet = Int64[],
              Start = String[],
              End = String[],
              alfa = Float64[],
              N = Int64[],
              density_factor = Float64[],
              update_period = Int64[],
              T = Float64[],
              k = Int64[],
              mode = String[],
              TotalTimeReduction = Float64[],
              SmartTimeReduction = Float64[],
              NotSmartTimeReduction = Float64[],
              Runtime = Float64[])

ParameterRange = 0.1:0.1:1.0
for element in ParameterRange
      for i in 1:5
      println("$element : $i")
      #Generating agents
      Agents = generate_agents(map_data, N, Start, End, element)[1]
      #Running base simulation - no V2I system
      BaseOutput = simulation_run("base", map_data, Agents)
      step_statistics = gather_statistics(getfield.(Agents,:smart),
                                          BaseOutput.TravelTimes,
                                          ITSOutput.TravelTimes)
      println(step_statistics)
      push!(ResultFrame, [mapfile, RoadSet,
                          string((Start[1].p1,Start[1].p2)), string((End[1].p1,End[1].p2)),
                          element, N, density_factor,
                          updt_period, T, k,
                          mode,
                          step_statistics.overall_time,
                          step_statistics.smart_time,
                          step_statistics.other_time,
                          runtime])
      end
end
CSV.write("RenoV2I.csv", ResultFrame)
