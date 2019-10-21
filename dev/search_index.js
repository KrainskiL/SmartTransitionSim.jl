var documenterSearchIndex = {"docs":
[{"location":"reference/#Reference-1","page":"Reference","title":"Reference","text":"","category":"section"},{"location":"reference/#","page":"Reference","title":"Reference","text":"CurrentModule = RSUOptimization\nDocTestSetup = quote\n    using SmartTransitionSim\nend","category":"page"},{"location":"reference/#Agent-generation-1","page":"Reference","title":"Agent generation","text":"","category":"section"},{"location":"reference/#","page":"Reference","title":"Reference","text":"Rect\nAgent\npick_random_node\ngenerate_agents","category":"page"},{"location":"reference/#SmartTransitionSim.Rect","page":"Reference","title":"SmartTransitionSim.Rect","text":"Rect type represent rectangle area on map\n\nFields\n\np1 : Latitude-Longitude coordinates of first rectangle corner\np2 : Latitude-Longitude coordinates of second rectangle corner\n\n\n\n\n\n","category":"type"},{"location":"reference/#SmartTransitionSim.Agent","page":"Reference","title":"SmartTransitionSim.Agent","text":"Agent type stores information about agents\n\nFields\n\nsmart : logical value indicating if agent use re-routing service\nstart_node : starting point of agent's route\nend_node : ending point of agent's route\nroute : array of nodes determining agent's route (may be changed by re-routing)\nstart_time : time after which agent starts moving\ntravel_time : time spent in simulation\nedge : current edge agent is on\nactive : indicates if agent is active in simulation\n\n\n\n\n\n","category":"type"},{"location":"reference/#SmartTransitionSim.pick_random_node","page":"Reference","title":"SmartTransitionSim.pick_random_node","text":"pick_random_node function is used to set starting and ending node of agents. Nodes are randomly chosen from set of provided areas.\n\nInput parameters\n\nOSMmap : OpenStreetMapX mapData object with road network data\nrects : vector of Rect types interpreted as a set of rectangle areas\nNodesSet : returning object switch\nfalse return random node from given area\ntrue return all nodes in given area for further processing\nseed : set seed for rand function\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.generate_agents","page":"Reference","title":"SmartTransitionSim.generate_agents","text":"generate_agents function creating vector of agents and returning travel time for initial routes travelled with maximal speed\n\nInput parameters\n\nOSMmap : OpenStreetMapX MapData object with road network data\nN : number of agents to be generated\nStartArea : vector of areas from which agents randomly pick starting point\nEndArea : vector of areas from which agents randomly pick ending point\nα : percentage of smart agents\nk : number of fastest routes used in k-path algorithm\nT : control variable for k-path algorithm probability distribution\nAvgStartTime : average agents start time in minutes\nk_routes_dict : dictionary with multiple shortest paths (values) between vertices (keys)\nseed : set seed for random function\n\n\n\n\n\n","category":"function"},{"location":"reference/#Rerouting-1","page":"Reference","title":"Rerouting","text":"","category":"section"},{"location":"reference/#","page":"Reference","title":"Reference","text":"k_shortest_path_rerouting!\nyen_a_star","category":"page"},{"location":"reference/#SmartTransitionSim.k_shortest_path_rerouting!","page":"Reference","title":"SmartTransitionSim.k_shortest_path_rerouting!","text":"k_shortest_path_rerouting! change agents current route according to traffic conditions\n\nInput parameters\n\nOSMmap : OpenStreetMapX MapData object with road network data\nk_routes_dict : dictionary with multiple shortest paths (values) between vertices (keys)\ninAgent : modified agent\nspeeds : current speed matrix\nmax_speeds : matrix with maximal speeds on edges\nk : number of fastest routes returned\nT : control variable for probability distribution\nupd_period : update period in seconds - if different from 0 enables rerouting based on distance travelled during one update period\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.yen_a_star","page":"Reference","title":"SmartTransitionSim.yen_a_star","text":"yen_a_star is modified version of LightGraphs yenkshortest_paths function\n\nInput parameters\n\ng : road network graph\nsource : source vertex\ntarget : target vertex\ndistmx : weight matrix\nK : number of routes to be calculated\n\n\n\n\n\n","category":"function"},{"location":"reference/#Traffic-model-1","page":"Reference","title":"Traffic model","text":"","category":"section"},{"location":"reference/#","page":"Reference","title":"Reference","text":"get_max_densities\nupdate_weights!\nupdate_weights_and_events!\ntraffic_constants\ninit_traffic_variables\nnext_edge\nupdate_event_agent!","category":"page"},{"location":"reference/#SmartTransitionSim.get_max_densities","page":"Reference","title":"SmartTransitionSim.get_max_densities","text":"get_max_densities calculate maximal traffic densities on all edges\n\nInput parameters\n\nOSMmap : OpenStreetMapX MapData object with road network data\ndensity_factor : road length reserved for one vehicle\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.update_weights!","page":"Reference","title":"SmartTransitionSim.update_weights!","text":"update_weights! change speeds in given speedmatrix for edges listed in newdensities\n\nInput parameters\n\nspeed_matrix : matrix with average speeds on edges\nnew_densities : dictionary with edges as keys and new traffic density as value\nmax_densities : matrix with maximal densitites on edges\nV_max : matrix with maximal speeds on edges\nV_min : minimal speed on road\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.update_weights_and_events!","page":"Reference","title":"SmartTransitionSim.update_weights_and_events!","text":"update_weights_and_events! update events time and weights in speed matrix\n\nInput parameters\n\ninAgents : set of agents created with generate_agents function\nagents_pos : vector with agents current position (edges)\nevents : vector with events time\nspeed_matrix : matrix with average speeds on edges\nedges : one or two edges that took part in an event\ndens : dictionary with current density on edges\nmax_densities : matrix with maximal densities\nV_max : matrix with speed limits\nV_min : minimal speed allowed on road\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.traffic_constants","page":"Reference","title":"SmartTransitionSim.traffic_constants","text":"traffic_constants create maximal traffic densities and speeds matrices\n\nInput parameters\n\nOSMmap : OpenStreetMapX MapData object with road network data\ndensity_factor : road length reserved for one vehicle\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.init_traffic_variables","page":"Reference","title":"SmartTransitionSim.init_traffic_variables","text":"init_traffic_variables create data used in calculating velocities change during simulation\n\nInput parameters\n\nOSMmap : OpenStreetMapX MapData object with road network data\nAgents : set of agents created with generate_agents function\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.next_edge","page":"Reference","title":"SmartTransitionSim.next_edge","text":"next_edge returns time required to reach next junction for one agent\n\nInput parameters\n\nAgent : single instance of Agent\nspeeds : current speeds matrix\nlengths : road lengths matrix\n\n\n\n\n\nnext_edge returns time required to reach next junction for set of agents\n\nInput parameters\n\nAgents : set of agents created with generate_agents function\nspeeds : current speeds matrix\nlengths : road lengths matrix\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.update_event_agent!","page":"Reference","title":"SmartTransitionSim.update_event_agent!","text":"update_event_agent! update densities matrix, progress agents to next edge and deactivate agents\n\nInput parameters\n\ninAgent : agent connected with occuring event\ncurr_time : current simulation time\ndensities : current traffic densitites dictionary\nvertices_map : mapping from nodes to vertices\n\n\n\n\n\n","category":"function"},{"location":"reference/#Simulation-1","page":"Reference","title":"Simulation","text":"","category":"section"},{"location":"reference/#","page":"Reference","title":"Reference","text":"simulation_run\ngather_statistics","category":"page"},{"location":"reference/#SmartTransitionSim.simulation_run","page":"Reference","title":"SmartTransitionSim.simulation_run","text":"simulation_run run traffic simulation in two modes.\n\nInput parameters\n\nmode : simulation model switch\n:base : simulation with regular agents only\n:smart : simulation with smart agents enabled\nOSMmap : OpenStreetMapX MapData object with road network data\ninAgents : set of agents created with generate_agents function\ndensity_factor : road length reserved for one vehicle\nk_routes_dict : dictionary with multiple shortest paths (values) between vertices (keys)\nU : period of weights updates\nT : distribution parameter in k-shortest path rerouting\nk : number of fastest routes generated in rerouting function\ndebug : debug messages switch\ntrack_avg_speeds : return average speeds on edges if true\n\n\n\n\n\n","category":"function"},{"location":"reference/#SmartTransitionSim.gather_statistics","page":"Reference","title":"SmartTransitionSim.gather_statistics","text":"gather_statistics return various measures describing TMS model quality\n\nInput parameters\n\nsmart_ind : vector with smart agent indicators\ntimes_base : agents travelling time in base scenario\ntimes_smart : agents travelling time in smart scenario\n\n\n\n\n\n","category":"function"},{"location":"#SmartTransitionSim.jl-1","page":"SmartTransitionSim.jl","title":"SmartTransitionSim.jl","text":"","category":"section"},{"location":"#","page":"SmartTransitionSim.jl","title":"SmartTransitionSim.jl","text":"Documentation for SmartTransitionSim.jl","category":"page"}]
}
