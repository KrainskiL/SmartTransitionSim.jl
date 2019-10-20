###################################
## Simulation models ##
###################################

function count_active_agents(agents::Vector{Agent})
    r = 0
    @simd for i = eachindex(agents)
        @inbounds r += agents[i].active
    end
    r
end

"""
`simulation_run` run traffic simulation in two modes.

**Input parameters**
* `mode` : simulation model switch
    * `:base` : simulation with regular agents only
    * `:smart` : simulation with smart agents enabled
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `inAgents` : set of agents created with generate_agents function
* `density_factor` : road length reserved for one vehicle
* `k_routes_dict` : dictionary with multiple shortest paths (values) between vertices (keys)
* `U` : period of weights updates
* `T` : distribution parameter in k-shortest path rerouting
* `k` : number of fastest routes generated in rerouting function
* `debug` : debug messages switch
* `track_avg_speeds` : return average speeds on edges if true
"""
function simulation_run(mode::Symbol,
                        OSMmap::MapData,
                        inAgents::Vector{Agent},
                        density_factor::Float64 = 5.0,
                        k_routes_dict::Dict{Tuple{Int,Int},Array{Vector{Int}}}=
                        Dict{Tuple{Int,Int},Array{Vector{Int}}}(),
                        U::Int = 100,
                        T::Float64 = 1.0,
                        k::Int = 3,
                        debug::Bool = false;
                        track_avg_speeds::Bool = false)
    if !in(mode, [:base,:smart]) error("Wrong mode specified.") end
    Agents = deepcopy(inAgents) #Creating working copy of agents
    Agents[1].start_time = 0.0
    Agents[1].active = true
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    curr_edges = [a.edge for a in Agents] #Initialize edges list
    if track_avg_speeds
        avg_speeds = deepcopy(speeds)
        tick = 1
    end
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = length(Agents)
    #Calculate initial time to nearest junction for all agents
    times_to_edge = next_edge(Agents, speeds, OSMmap.w)
    times_to_start = [a.start_time for a in Agents] #Get agents starting times
    times_to_start[1] = Inf
    event_update = Inf
    #Loop until all agents are deactivated
    while active != 0 || !all(times_to_start.==Inf)
        steps += 1
        #Calculate next event time and update sim clock
        event_edge, IDe = findmin(times_to_edge)
        event_start, IDs = findmin(times_to_start)
        if mode == :smart
            event_update = next_update = (simtime ÷ U + 1) * U - simtime
        end
        min_event, event_type = findmin([event_edge, #1
                                        event_start, #2
                                        event_update]) #3
        simtime += min_event #Increase total simulation time
        times_to_edge .-= min_event #Move agents forward to event time
        times_to_start .-= min_event
        if event_type == 1 #Switching edges
            vAgent = Agents[IDe] #Pick agent for which event is occuring
            #Process agent for which event is occuring
            changed_edges = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
            length(changed_edges) == 1 ? curr_edges[IDe] = (0,0) : curr_edges[IDe] = changed_edges[2]
            #Update speeds and correct events time
            update_weights_and_events!(Agents, curr_edges, times_to_edge, speeds,
                changed_edges, densities, max_densities, max_speeds)
            vAgent.active ? times_to_edge[IDe] = next_edge(vAgent, speeds, OSMmap.w) : times_to_edge[IDe] = Inf
        elseif event_type == 2 #New agent joins traffic
            Agents[IDs].active = true
            times_to_start[IDs] = Inf
            times_to_edge[IDs] = next_edge(Agents[IDs], speeds, OSMmap.w)
            c_edge = Agents[IDs].edge
            haskey(densities, c_edge) ? densities[c_edge] += 1 : densities[c_edge] = 1
        elseif event_type == 3 #Velocities update
            if debug
                update = Int(simtime ÷ U)
                smartAgents = sum([a.smart*a.active for a in Agents])
                print("Update: $(update) | Smart agents: $(smartAgents) ")
            end
            #Process rerouting for smart agents
            for a in Agents
                a.smart && a.active &&
                k_shortest_path_rerouting!(OSMmap, k_routes_dict, a, speeds, max_speeds, k, T, U)
            end
            debug && println("Finished")
        end
        #Update average speeds every 30 secs
        if track_avg_speeds && simtime > tick*30
            #Iterative mean
            tick +=1
            avg_speeds = ((tick-1)/tick)*avg_speeds+(1/tick)*speeds
        end
        active = count_active_agents(Agents)
    end
    times = getfield.(Agents,:travel_time)
    if track_avg_speeds
        output_tuple = (Steps = steps,
                    Simtime = simtime,
                    TravelTimes = times,
                    AvgSpeeds = avg_speeds)
    else
        output_tuple = (Steps = steps,
                    Simtime = simtime,
                    TravelTimes = times)
    end
    return output_tuple
end

"""
`gather_statistics` return various measures describing TMS model quality

**Input parameters**
* `smart_ind` : vector with smart agent indicators
* `times_base` : agents travelling time in base scenario
* `times_smart` : agents travelling time in smart scenario
"""
function gather_statistics(smart_ind::BitArray{1},
                            times_base::Vector{Float64},
                            times_smart::Vector{Float64})
    overall_time = (sum(times_base) - sum(times_smart))/sum(times_base)
    smart_time = (sum(times_base[smart_ind]) - sum(times_smart[smart_ind]))/sum(times_base[smart_ind])
    other_time = (sum(times_base[.!smart_ind]) - sum(times_smart[.!smart_ind]))/sum(times_base[.!smart_ind])
    statistics_tuple = (
        overall_time = round(overall_time, digits=3),
        smart_time = round(smart_time, digits=3),
        other_time = round(other_time, digits=3),
        avg_base = round(StatsBase.mean(times_base), digits=2),
        avg_overall_V2I = round(StatsBase.mean(times_smart), digits=2),
        avg_smart_V2I = round(StatsBase.mean(times_smart[smart_ind]), digits=2),
        avg_regular_V2I = round(StatsBase.mean(times_smart[.!smart_ind]), digits=2)
                        )
    return statistics_tuple
end

"""
`run_parameter_analysis` run base and V2I simulation scenario to compare agents performance, returns comparison statistics

**Input parameters**
* `GridElement` : element from parameters grid to be used in simulation
* `ParamGrid` : array with parameters values
* `Start` : vector of areas from which agents randomly pick starting point
* `End` : vector of areas from which agents randomly pick ending point
* `density_factor` : road length reserved for one vehicle
* `K_Paths_Dict` : dictionary with multiple shortest paths (values) between vertices (keys)
* `mapdata` : OpenStreetMapX MapData object with road network data
"""
function run_parameter_analysis(GridElement::Int,
                                ParamGrid::Vector{Tuple{Int,Float64,Int,Int,Float64,Int,Float64}},
                                Start::Vector{Rect}, End::Vector{Rect}, density_factor::Float64,
                                K_Paths_Dict::Dict{Tuple{Int,Int},Array{Vector{Int}}},
                                mapdata::OpenStreetMapX.MapData)
  startTime = time()
  p = ParamGrid[GridElement]
  #Generating agents
  Agents = generate_agents(mapdata, p[3], Start, End, p[2], p[6], p[5], p[7], K_Paths_Dict)
  #Running base simulation - no V2I system
  BaseOutput = simulation_run(:base, mapdata, Agents, density_factor)
  #Running simulation with smart agents - V2I system enabled
  SmartOutput = simulation_run(:smart, mapdata, Agents, density_factor,
                              K_Paths_Dict, p[4], p[5], p[6])
  step_stat = gather_statistics(getfield.(Agents,:smart),
                                      BaseOutput.TravelTimes,
                                      SmartOutput.TravelTimes)
  runtime = time()-startTime
  print("$(GridElement),$(p[1]),$(p[2]),$(p[3]),$(p[4]),$(p[5]),$(p[6]),$(p[7]),")
  print("$(round(runtime,digits = 2)),$(step_stat[1]),$(step_stat[2]),$(step_stat[3]),")
  println("$(step_stat[4]),$(step_stat[5]),$(step_stat[6]),$(step_stat[7])")
end

"""
`run_parameter_analysis` run base and V2I simulation scenario to compare agents performance, returns comparison statistics

**Input parameters**
* `GridElement` : element from parameters grid to be used in simulation
* `ParamGrid` : array with parameters values
* `AgentsVec` : vector with agents used in simulation
* `density_factor` : road length reserved for one vehicle
* `K_Paths_Dict` : dictionary with multiple shortest paths (values) between vertices (keys)
* `mapdata` : OpenStreetMapX MapData object with road network data
"""
function run_parameter_analysis(GridElement::Int,
                                ParamGrid::Vector{Tuple{Int,Float64,Int,Int,Float64,Int,Float64}},
                                AgentsVec::Vector{Agent}, density_factor::Float64,
                                K_Paths_Dict::Dict{Tuple{Int,Int},Array{Vector{Int}}},
                                mapdata::OpenStreetMapX.MapData)
  startTime = time()
  p = ParamGrid[GridElement]
  #Sample N agents from supplied vector and adjust smart agents number
  Agents = deepcopy(sample(AgentsVec, p[3], replace = false))
  N_int= Int(ceil(p[3]*p[2]))
  smart_ind = [trues(N_int); falses(p[3]-N_int)]
  for i in 1:p[3] Agents[i].smart = smart_ind[i] end
  #Running base simulation - no V2I system
  BaseOutput = simulation_run(:base, mapdata, Agents, density_factor)
  #Running simulation with smart agents - V2I system enabled
  SmartOutput = simulation_run(:smart, mapdata, Agents, density_factor,
                              K_Paths_Dict, p[4], p[5], p[6])
  step_stat = gather_statistics(getfield.(Agents,:smart),
                                      BaseOutput.TravelTimes,
                                      SmartOutput.TravelTimes)
  runtime = time()-startTime
  print("$(GridElement),$(p[1]),$(p[2]),$(p[3]),$(p[4]),$(p[5]),$(p[6]),$(p[7]),")
  print("$(round(runtime,digits = 2)),$(step_stat[1]),$(step_stat[2]),$(step_stat[3]),")
  println("$(step_stat[4]),$(step_stat[5]),$(step_stat[6]),$(step_stat[7])")
end
