"""
`print_map_statistics` prints basic map file properties and characteristics
of road network (OpenStreetMapX.MapData type) built on top of that

**Fields**
* `datapath` : path to map file
* `mapfile` : map file name
* `roadclass` : highest road classe available on the map

"""
function print_map_statistics(datapath::String, mapfile::String, roadclass::Int)
    #Load map
    map_data = get_map_data(datapath, mapfile, use_cache=false; road_levels = Set(1:roadclass));
    #Map file info
    println("Map name: $mapfile")
    println("Size: $(filesize(datapath*mapfile)) B")
    filehash=hash(read(datapath*mapfile))
    println("Map file hash generated with Base.hash function: $(string(filehash,base=16))")
    #Map load statement
    println("\nMap statistics generated for map loaded with statement:")
    println("OpenStretMapX.get_map_data('$(datapath)', '$(mapfile)', use_cache=false; road_levels = Set(1:$(roadclass)))")

    #Bounds
    println("\n---Map bounds (LLA coordinates)---")
    println("MinY = $(map_data.bounds.min_y)");
    println("MaxY = $(map_data.bounds.max_y)");
    println("MinX = $(map_data.bounds.min_x)");
    println("MaxY = $(map_data.bounds.max_y)");
    #Graph
    println("\n---Graph characteristics---")
    print("No. of vertices: "); println(length(map_data.v))
    print("No. of edges: "); println(length(map_data.e))

    #Transportation network
    println("\n---Edges characteristics---") #2176
    get_velocities = OpenStreetMapX.get_velocities(map_data)
    velo_dict = sort(countmap(get_velocities.nzval));
    println("\nNumber of edges with given speed limits:")
    for (k,v) in velo_dict
        println("$(round(k*3.6, digits=2)) km/h: $v edges")
    end

    println("\nNumber of edges with given lanes:") #2176
    roads_lanes = Dict{Int,Int}()
    for r in map_data.roadways
        OpenStreetMapX.haslanes(r) ? lanes = OpenStreetMapX.getlanes(r) : lanes = 1
        roads_lanes[r.id] = lanes
    end
    segments = OpenStreetMapX.find_segments(map_data.nodes, map_data.roadways, map_data.intersections)
    filtered_segments = [[s.node0, s.node1, roads_lanes[s.parent]] for s in segments]
    unique_segments = unique(filtered_segments)
    lanes = [s[3] for s in unique_segments]
    lanes_dict = sort(countmap(lanes))
    for (k,v) in lanes_dict
        if k==1
            println("$k lane: $v edges")
        else
            println("$k lanes: $v edges")
        end
    end

    println("\n---Edges length distribution---") #2176
    edge_len = map_data.w.nzval
    length_quan = quantile(edge_len,0:0.1:1)
    println()
    print(summarystats(edge_len))
    return true
end
