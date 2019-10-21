##################################
## Rerouting module ##
##################################

"""
`k_shortest_path_rerouting!` change agents current route according to traffic conditions

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `k_routes_dict` : dictionary with multiple shortest paths (values) between vertices (keys)
* `inAgent` : modified agent
* `speeds` : current speed matrix
* `max_speeds` : matrix with maximal speeds on edges
* `k` : number of fastest routes returned
* `T` : control variable for probability distribution
* `upd_period` : update period in seconds - if different from 0 enables rerouting based on distance travelled during one update period
"""
function k_shortest_path_rerouting!(OSMmap::MapData,
                                    k_routes_dict::Dict{Tuple{Int,Int},Array{Vector{Int}}},
                                    inAgent::Agent,
                                    speeds::AbstractMatrix,
                                    max_speeds::AbstractMatrix,
                                    k::Int,
                                    T::Float64,
                                    upd_period::Int = 0,
                                    seed::Int = -1)
    #Assigning start and end vertices for rerouting
    route = inAgent.route
    #Exit function if route length is 3
    if length(route) == 3 return end
    vx1 = OSMmap.v[route[2]]
    vx2 = OSMmap.v[route[end]]
    avg_speed_ratio = 0.0
    #Calculating number of vertices travelled during update period
    if upd_period != 0
        cutoff = 2
        travel_time = 0.0
        while cutoff+1 < length(route) && travel_time < upd_period
            v1 = OSMmap.v[route[cutoff]]
            v2 = OSMmap.v[route[cutoff+1]]
            c_speed = speeds[v1,v2]
            travel_time += OSMmap.w[v1,v2]/c_speed
            avg_speed_ratio = ((cutoff-2)/(cutoff-1))*avg_speed_ratio +
                (1/(cutoff-1))*c_speed/max_speeds[v1,v2]
            cutoff+=1
        end
        # Manual override of cutoff - disabling rerouting on 1-segment route fragments
        if cutoff == 3
            cutoff = 4
            if cutoff!= length(route)
                v1 = OSMmap.v[route[cutoff]]
                v2 = OSMmap.v[route[cutoff+1]]
                avg_speed_ratio = (2/3)*avg_speed_ratio +
                    (1/3)*speeds[v1,v2]/max_speeds[v1,v2]
            end
        end
        vx2 = OSMmap.v[inAgent.route[cutoff]]
    end
    #Reroute only if traffic is congested in predicted route segment
    if avg_speed_ratio < 0.9
        #Check if k-shortest paths between vertices is already calculated
        if !haskey(k_routes_dict,(vx1,vx2))
            k_routes_dict[(vx1,vx2)] = yen_a_star(OSMmap.g, vx1, vx2, OSMmap.w, 6).paths
        end
        #Extract 6 shortest paths and calculate travelling time for each
        k_paths = k_routes_dict[(vx1,vx2)]
        time = zeros(length(k_paths))
        for p in 1:length(k_paths)
            path = k_paths[p]
            for i in 1:(length(path)-1)
                time[p] += OSMmap.w[path[i],path[i+1]]/speeds[path[i],path[i+1]]
            end
        end
        #Remove paths with travelling time longer than doubled minimal time
        adjusted_k = sum(time.<=2*minimum(time))
        k = min(adjusted_k,k)
        #Pick k fastest paths
        picked_paths = k_paths[sortperm(time)[1:k]]
        times = time[sortperm(time)[1:k]]
        #Normalize k-paths travelling time
        norm_time = times/maximum(times)
        #Calculate probability of being picked for every route
        exp_ntime = exp.(-norm_time/T)
        probs = exp_ntime/sum(exp_ntime)
        #Assign new route
        if seed != -1 Random.seed!(seed) end
        new_path = sample(picked_paths, StatsBase.weights(probs))
        nodes_new_path = map(i-> OSMmap.n[i], new_path)
        if upd_period != 0 && cutoff!= length(route)
            inAgent.route = [inAgent.route[1]; nodes_new_path ; inAgent.route[(cutoff+1):end]]
        else
            inAgent.route = [inAgent.route[1]; nodes_new_path]
        end
    end
end

"""
`yen_a_star` is modified version of LightGraphs yen_k_shortest_paths function

**Input parameters**
* `g` : road network graph
* `source` : source vertex
* `target` : target vertex
* `distmx` : weight matrix
* `K` : number of routes to be calculated
"""
function yen_a_star(g::AbstractGraph,
    source::Int,
    target::Int,
    distmx::AbstractMatrix=weights(g),
    K::Int=1)

    dj = OpenStreetMapX.a_star_algorithm(g, source, target, distmx)
    path = dj[1]

    dists = Array{Float64,1}()
    push!(dists, dj[2])
    A = [path]
    B = DataStructures.PriorityQueue()
    gcopy = deepcopy(g)

    for k = 1:(K - 1)
        for j = 1:(length(A[k])-1)
            spurnode = A[k][j]
            rootpath = A[k][1:j]
            edgesremoved = Array{Tuple{Int,Int},1}()
            for ppath in A
                if length(ppath) > j && rootpath == ppath[1:j]
                    u = ppath[j]
                    v = ppath[j + 1]
                    if has_edge(gcopy, u, v)
                        rem_edge!(gcopy, u, v)
                        push!(edgesremoved, (u, v))
                    end
                end
            end
            distrootpath = 0.
            for n = 1:(length(rootpath) - 1)
                u = rootpath[n]
                nei = copy(neighbors(gcopy, u))
                for v in nei
                    rem_edge!(gcopy, u, v)
                    push!(edgesremoved, (u, v))
                end
                v = rootpath[n + 1]
                distrootpath += distmx[u, v]
            end
            djspur = OpenStreetMapX.a_star_algorithm(gcopy, spurnode, target, distmx)
            spurpath = djspur[1]
            if !isempty(spurpath)
                pathtotal = [rootpath[1:(end - 1)]; spurpath]
                distpath  = distrootpath + djspur[2]
                if !haskey(B, pathtotal)
                    enqueue!(B, pathtotal, distpath)
                end
            end
            for (u, v) in edgesremoved
                add_edge!(gcopy, u, v)
            end
        end
        isempty(B) && break
        mindistB = peek(B)[2]
        push!(dists, peek(B)[2])
        push!(A, dequeue!(B))
    end

    return LightGraphs.YenState(dists, A)
end
