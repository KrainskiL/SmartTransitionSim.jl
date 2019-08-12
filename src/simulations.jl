###################################
## Simulation models ##
###################################
"""
`simulation_run` run traffic simulation.

**Input parameters**
* `mode` : simulation model switch
    * `base` : simulation with regular agents only
    * `smart` : simulation with smart agents enabled
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `U` : period of weights updates
* `T` : distribution parameter in k-shortest path rerouting
* `k` : number of fastest routes generated in rerouting function
* `density_factor` : road length reserved for one vehicle
* `debug_level` : debug messages switch
    * 0 and 1 - none
    * 2 - updates info
    * 3 - active agents info
"""
function simulation_run(mode::String,
                        OSMmap::MapData,
                        inAgents::Vector{Agent},
                        U::Int64 = 100,
                        T::Float64 = 1.0,
                        k::Int64 = 3;
                        density_factor::Float64 = 5.0,
                        debug_level::Int64 = 0)
    mode = lowercase(mode)
    if !in(mode, ["base","smart"]) error("Wrong mode specified.") end
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug_level > 2 modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end
    #Calculate initial time to nearest junction for all agents
    times_to_event = next_edge(Agents, speeds, OSMmap.w)
    #Loop until all agents are deactivated
    while active != 0
        steps += 1
        #Calculate next event time
        event_time, ID = findmin(times_to_event)
        if mode != "base"
            #Calculate time to next weights update
            next_update = (simtime ÷ U + 1) * U - simtime
            #Check if weight updates occur before event_time
            if next_update < event_time
                if debug_level > 1 update_nr = Int(simtime ÷ U + 1) end
                times_to_event .-= next_update
                #Reroute updated agents
                for a in Agents
                    a.smart && k_shortest_path_rerouting!(OSMmap, a, speeds, k, T)
                end
                simtime += next_update #Increase simulation time
                debug_level > 1 && println("Finished")
                continue #Skip to next event
            end
        end
        simtime += event_time #Increase total simulation time
        vAgent = Agents[ID] #Pick agent for which event is occuring
        times_to_event .-= event_time #Move agents forward to event time
        #Process agent for which event is occuring
        changed_edges = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        if !vAgent.active times_to_event[ID] = Inf end
        #Update speeds
        speed_factors = update_weights!(speeds, changed_edges, densities, max_densities, max_speeds)
        #Correct events time due to speed changes
        event_time_correction!(Agents, changed_edges, speed_factors, times_to_event)
        if vAgent.active times_to_event[ID] = next_edge(vAgent, speeds, OSMmap.w) end
        active = sum(getfield.(Agents,:active))
        if debug_level > 2 && length(changed_edges) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
    end
    times = getfield.(Agents,:travel_time)
    output_tuple = (Steps = steps,
                    Simtime = simtime,
                    TravelTimes = times)
    return output_tuple
end

"""
`gather_statistics` return various measures describing TMS model quality

**Input parameters**
* `smart_ind` : vector with true flags for smart agents
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
        other_time = round(other_time, digits=3)
                        )
    return statistics_tuple
end
