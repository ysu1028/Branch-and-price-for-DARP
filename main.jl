using JuMP, Gurobi
using DelimitedFiles
using DataStructures
using DataFrames
using JSON
using Random
using StatsBase
using Distributions
using Combinatorics
using MathOptInterface
using Plots
const MOI = MathOptInterface
import Base: push!
import Base: run

#veh_list = [2,2,2,3,3,3,3,4,4,4,4,4,5,5]
#user_list = [16,20,24,18,24,30,36,16,24,32,40,48,40,50]
veh_list = [2]
user_list = [16]

#gama = [0.1,0.1,0.1,0.1,0.1]
gama = [0.0,0.0,0.0,0.0,0.0]

current_path = @__DIR__
current_path = string(current_path,"/")
resultspath = string(current_path,"results/")

numb_user = user_list[1]
numb_veh = veh_list[1]

include(string(current_path,"instance.jl"))
numb_nodes,numb_recharging,T_p,s,q,e,l,art_origin_depot,art_end_depot,charging_station,m,VC,BC,alfa,beta,weight,H_rc,h_rc,d,t,n,k,K,P_u,D_u,N,P_d,D_d,F,V = Instance(numb_veh,numb_user,gama)

include(string(current_path,"utils/basic_function.jl"))
include(string(current_path,"utils/preprocessing.jl"))
include(string(current_path,"utils/neighborhood_prunning.jl"))
sequences_list = Dict()
for i in P_u
    sequences_list[i] = []
    sequences = all_sequences(i,weight)
    for j in 1:length(sequences)
        push!(sequences_list[i], sequences[j])
    end
end
all_segments = []
for i in P_u
  for j in 1:length(sequences_list[i])
    sequence = sequences_list[i][j]
    push!(all_segments,sequence[6])
  end
end
conflict = create_conflict_graph()

# utils
# start from the empty set as initial solution
include(string(current_path,"utils/initial_solution.jl"))
include(string(current_path,"utils/branching.jl"))
include(string(current_path,"utils/milp.jl"))
include(string(current_path,"utils/colgen.jl"))
include(string(current_path,"utils/filter.jl"))
# single objective BnP module
include(string(current_path,"BranchAndPrice.jl"))
include(string(current_path,"bnp/implementation.jl"))

total_time = @elapsed opt, opt_sol, total_time, cols, cost_cols, total_nodes, remain_nodes = run_BP([], [], weight, [])
