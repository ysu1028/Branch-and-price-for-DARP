using DelimitedFiles
using DataStructures
using DataFrames
using JSON

"""
    Instance{P}

A DARP instance gathers all information needed to define the problem.

numb_nodes,numb_recharging,T_p,s,q,e,l,charging_station,m,VC,BC,alfa,beta,H_rc,h_rc,
d,t,n,k,K,P_u,D_u,N,P_d,D_d,F,V

# Fields (explantion for some of them)
- `T_p::Int`: planning horizon .
- `q::Vector{Float64}`: load at each node.
- `s::Vector{Float64}`: service time at each node.
- `e::Vector{Float64}: earliest service begin time`
"""
Base.@kwdef struct Instance
    numb_user :: Int
    numb_veh :: Int
    numb_nodes :: Int
    numb_recharging :: Int
    T_p :: Int
    s :: Vector{Float64} # service time
    q :: Vector{Float64} # load
    e :: Vector{Float64} # earliest time window
    l :: Vector{Float64} # latest tiome window
    charging_station :: Vector{Int}
    m :: Vector{Int} # maximum user ride time
    VC :: Vector{Int} #maximum vehicle capacity
    BC :: Vector{Float64} # total battery capacity
    alfa :: Vector{Float64} # discharge rate
    beta :: Float64
    weight:: Vector{Float64}
    H_rc :: Float64
    h_rc :: Matrix{Float64}
    d :: Matrix{Float64}
    t :: Matrix{Float64}
    n :: Int
    k :: Int
    K :: UnitRange{Int}
    P_u :: UnitRange{Int}
    D_u :: UnitRange{Int}
    N :: Vector{Int}
    P_d :: Vector{Int}
    D_d :: Vector{Int}
    F :: Vector{Int}
    V :: Vector{Int}
end

function Instance(numb_veh::Int, numb_user::Int, gama::Vector{Float64})
    global current_path
    # read data
    filepath = string(current_path,"data/a",numb_veh,"-",numb_user,"-0.7.txt")
    data = readdlm(filepath, '\t', '\n')
    numb_style = 1 #can be re-defined
    numb_nodes = parse(Int64,split(data[1])[8])
    numb_veh = parse(Int64,split(data[1])[1])
    numb_user = parse(Int64,split(data[1])[2])
    numb_recharging = parse(Int64,split(data[1])[5])
    T_p = parse(Int64,split(data[1])[7]) # planning horizon in minutes
    Node_data = zeros(numb_nodes,7)
    for i in 1:numb_nodes
        for j in 1:7
            a = split(data[i+1])[j]
            Node_data[i,j] = parse(Float64, a)
            #push!(Node_data, [parse(Float32, a) for a in split(data[i+1])])
        end
    end
    Node_id = Node_data[:,1]
    lat = Node_data[:,2] # latitude
    long = Node_data[:,3] # longitude
    s = Node_data[:,4] # service time on node
    q = Node_data[:,5] # load on node
    e = Node_data[:,6] # earliest time of time window of users and depors in minutes
    l = Node_data[:,7] # latest time of time window of users and depots in minutes
    common_origin_depot = data[numb_nodes+2]
    common_end_depot = data[numb_nodes+3]
    art_origin_depot = [parse(Int64,a) for a in split(data[numb_nodes+4])] # 2 artificial origin depots
    art_end_depot = [parse(Int64,a) for a in split(data[numb_nodes+5])]
    charging_station = []
    if gama[1] != 0.0
        charging_station = [parse(Int64,a) for a in split(data[numb_nodes+6])] # 3 charging stations
    end
    m = [parse(Int64,a) for a in split(data[numb_nodes+7])] # maximum user ride time (16 users) in minutes

    # vehicle related parameters
    VC = [parse(Int64,a) for a in split(data[numb_nodes+8])] # vehicle capacity
    BC_i = [parse(Float64,a) for a in split(data[numb_nodes+9])] # vehicles initial battery inventory
    BC = [parse(Float64,a) for a in split(data[numb_nodes+10])] # effective battery capacity in kwh
    alfa = [parse(Float64,a) for a in split(data[numb_nodes+12])] # recharging rates at charging stations, kwh/min
    beta = data[numb_nodes+13] # vehicle discharging rate, kwh/min
    weight = [parse(Float64,a) for a in split(data[numb_nodes+14])]
    # distance calculation in km
    d = zeros(numb_nodes, numb_nodes)
    for i in 1:numb_nodes
        for j in 1:numb_nodes
            d[i,j] = sqrt((lat[i]-lat[j])^2+(long[i]-long[j])^2)
        end
    end
    # considering only one driving style in the first place
    t = d
    # convert the travel time into the corresponding recharging time
    h_rc = t*beta/alfa[1]
    H_rc = BC[1]/alfa[1]
    if gama[1] == 0.0
        H_rc = H_rc*1000
    end

    # set definition
    n = numb_user # 16 customer in total
    k = numb_veh # 2 vehicles
    K = 1:k # vehicle set
    P_u = 1:n
    D_u = 1+numb_user:n+numb_user
    N = union(P_u, D_u)
    # common depots
    depot_origin = common_origin_depot
    depot_end = common_end_depot
    # artificial depots
    P_d = art_origin_depot
    D_d = art_end_depot
    # recharging stations
    F = charging_station
    V = union(N,P_d,D_d,F) # all nodes set

    # time window tightening
    for i in P_u
        e[i] = round(max(e[i],e[numb_user+i]-m[i]-s[i]),digits=2)
        l[i] = round(min(l[numb_user+i]-t[i,numb_user+i]-s[i],l[i]),digits=2)
        e[numb_user+i] = round(max(e[numb_user+i],e[i]+t[i,numb_user+i]+s[i]),digits=2)
        l[numb_user+i] = round(min(l[i]+m[i]+s[i],l[numb_user+i]),digits=2)
    end

    # tightening recharging station time windows
    for i in F
        e[i] = round(minimum(e[j] + t[j,i] for j in P_d),digits=2)
        l[i] = round(maximum(T_p - t[i,j] for j in D_d), digits=2)
    end

    # tightening depot time windows

    for i in P_d
        e[i] = max(0,round(minimum(e[j]- t[i,j] for j in P_u),digits=2))
    end
    for i in D_d
        l[i] = min(l[i],round(maximum(l[j] + s[j] + t[j,i] for j in union(D_u,F)),digits=2))
    end

    return numb_nodes,numb_recharging,T_p,s,q,e,l,art_origin_depot,art_end_depot,charging_station,m,VC,BC,alfa,beta,weight,H_rc,h_rc,d,t,n,k,K,P_u,D_u,N,P_d,D_d,F,V
end

function generate_instance_as_struct(numb_veh::Int, numb_user::Int,gama::Vector{Float64})
    numb_nodes,numb_recharging,T_p,s,q,e,l,art_origin_depot,art_end_depot,charging_station,m,VC,BC,alfa,beta,weight,H_rc,h_rc,d,t,n,k,K,P_u,D_u,N,P_d,D_d,F,V = Instance(numb_veh,numb_user,gama)
    return Instance(;numb_veh, numb_user,numb_nodes,numb_recharging,T_p,s,q,e,l,art_origin_depot,art_end_depot,charging_station,m,VC,BC,alfa,beta,H_rc,h_rc,d,t,n,k,K,P_u,D_u,N,P_d,D_d,F,V)
end
