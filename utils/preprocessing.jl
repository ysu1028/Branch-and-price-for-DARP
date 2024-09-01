# preprocessing of labeling algorithm -> avoid redundance computation
# Objective: from each pickup node -> output: all the feasible sequence
####################################################################################################
function sequence_update(node1,time1,B_list1,prev1)
    feasibility = true
    delta_t = time1-l[node1]
    for i in 1:length(prev1)
        B_list1[i] -= delta_t
        if B_list1[i] < e[prev1[i]]
            return B_list1,false
        end
    end
    return B_list1,true
end

function sequence_feasibility(sequence1,node2)
    node1,cap1,time1,energy1,prev1,B_list = sequence1[1:6]
    feasibility = false
    time2 = max(time1 + t[node1,node2] + s[node1], e[node2])
    # B_list2,prev2
    B_list2 = deepcopy(B_list)
    push!(B_list2,time2)
    prev2 = deepcopy(prev1)
    push!(prev2,node2)
    if time2 > l[node2]
        B_list2, fes= sequence_update(node2,time2,B_list2,prev2)
        if fes == false
            return false
        else
            time2 = B_list2[end]
            feasibility = true
        end
    end
    # judgement1: vehicle capacity, battery
    if cap1 + q[node2] <= VC[1]
        feasibility = true
    else
        return false
    end

    if energy1 + h_rc[node1,node2] <= H_rc
        feasibility = true
    else
        return false
    end

    # judgement3: according to DARP characteristics
    if node2 in D_u && node2 - numb_user ∉ prev1
        return false
    end
    # judgement5: maximum user ride time constraint
    if node2 in D_u
        pickup_idx = find_index(node2-numb_user,prev2)
        D_pickup = B_list2[pickup_idx] + s[prev2[pickup_idx]]
        # 有人点尽早开始服务
        B_dropoff = max(e[node2],time2)
        if B_dropoff - D_pickup <= m[node2-numb_user]
            feasibility = true
        else
            return false
        end
    end
    return feasibility
end
# update excess user ride time when arriving the next zero-load node
function sequence_rt(prev1,B_list)
    ERT = 0
    for i in 1:length(prev1)
        if prev1[i] in P_u
            D_pickup = B_list[i]+s[prev1[i]]
            B_dropoff = B_list[find_index(prev1[i]+numb_user,prev1)]
            RT = B_dropoff-D_pickup
            ERT += RT-t[prev1[i],prev1[i]+numb_user]
        end
    end
    return ERT
end

# for sequence transported 3 passengers, call solver to solve
function three_passengers_case(partial_route,B_first)
    pickup = [i for i in partial_route if i in P_u && i != partial_route[end]]
    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model,"TimeLimit",300)
    # model = Model(CPLEX.Optimizer)
    # set_optimizer_attribute(model,"CPX_PARAM_TILIM",300)
    #@variable(model,x[i in partial_route, j in partial_route], Bin)
    @variable(model, U[i in partial_route]) # service begin time at node i
    @variable(model, R[i in pickup]) # excess ride time of user i
    @objective(model, Min, sum(R[i] for i in pickup))
    # constraint
    # time window related constraints
    @constraint(model, cons1[i in pickup], U[i]+s[i]+t[i,numb_user+i] <= U[numb_user+i])
    @constraint(model, cons2[i in partial_route], U[i] >= e[i])
    @constraint(model, cons2a, U[partial_route[1]] >= B_first)
    @constraint(model, cons3[i in partial_route], U[i] <= l[i])
    @constraint(model, cons4[i in pickup], R[i] >= U[numb_user+i] - U[i] - s[i] - t[i,numb_user+i])
    @constraint(model, cons5[i in pickup], U[numb_user+i] - U[i] - s[i] <= m[i])
    @constraint(model, cons6[i in partial_route[1:end-1]], U[i]+s[i]+t[i,partial_route[find_index(i,partial_route)+1]]<=U[partial_route[find_index(i,partial_route)+1]])
    @constraint(model, cons7[i in partial_route], U[i] >= 0)
    @constraint(model, cons8[i in pickup], R[i] >=0)
    optimize!(model)
    JuMP.optimize!(model)
    obj = JuMP.objective_value(model)
    service_begin = []
    excess_ride = []
    for i in partial_route
        push!(service_begin,JuMP.value.(U[i]))
    end
    for i in pickup
        push!(excess_ride,JuMP.value.(R[i]))
    end
    return obj,service_begin,excess_ride
end

# when arrive at the next zero-load node，optimize excess user ride time
function sequence_opt(prev1,B_list)
    B_list1 = deepcopy(B_list)
    # 上一个无人点的index
    idx_zero = 1
    cap = [q[prev1[idx_zero]]]
    for i in idx_zero+1:length(prev1)
        push!(cap,cap[i-idx_zero]+q[prev1[i]])
    end
    max_load = maximum(cap)
    idx_max_load = find_index(max_load,cap)

    # not earlier than the earliest time window
    delta = [B_list[idx_zero] - e[prev1[1]]]
    for i in idx_zero+1:length(prev1)
        delta_t = B_list[i]-e[prev1[i]]
        if delta_t > 0
            push!(delta,delta_t)
        else # 
            push!(delta,B_list[i-1]+t[prev1[i-1],prev1[i]]+s[prev1[i-1]] - B_list[i])
        end
    end
    # for those who arrive earlier than the earliest time window
    idx_early = []
    for i in idx_zero:length(prev1)
        if delta[i-idx_zero+1] < 0
            push!(idx_early,i-idx_zero+1) # i-idx_zero+1
        end
    end
    
    # case1: one passenger
    if max_load == 1
        if minimum(delta) > 0
            moving_forward = max(0,minimum(delta))
            for i in idx_zero:length(prev1)
                B_list[i] -= moving_forward
            end
        end 
    end
    # case2: two passengers
    if max_load == 2
        if minimum(delta) > 0
            moving_forward = max(0,minimum(delta))
            for i in idx_zero:length(prev1)
                B_list[i] -= moving_forward
            end
        else
            idx_need_to_delay = idx_zero+idx_max_load-1
            delay_supply = l[prev1[idx_need_to_delay]]-B_list[idx_need_to_delay]
            delay_needed = abs(delta[idx_max_load+1])
            if idx_max_load+1 in idx_early
                println(prev1)
                println(B_list)
                B_list[idx_need_to_delay] += min(delay_supply,delay_needed)
            end
        end
    end
    # case 3: three passengers or more -> call solver
    if max_load == 3
        if minimum(delta) > 0
            moving_forward = max(0,minimum(delta))
            for i in idx_zero:length(prev1)
                B_list[i] -= moving_forward
            end
        else
            println(prev1)
            println(B_list)
            total_ert,service_time_list,ert = three_passengers_case(prev1[idx_zero:end],B_list[idx_zero])
            B_list[idx_zero:end] = service_time_list
        end
    end
    return B_list,B_list1
end

function sequence_extension(sequence1)
    node1,cap1,time1,energy1,prev,B_list = sequence1[1:6]
    extended_route = []
    for i in setdiff(N,prev)
        node2 = i
        if sequence_feasibility(sequence1,node2) == true
            cap2 = cap1 + q[node2]
            B_list2,prev2 = deepcopy(B_list),deepcopy(prev)
            time2 = max(time1+t[node1,node2]+s[node1],e[node2])
            push!(B_list2,time2)
            push!(prev2,node2)
            energy2 = energy1 + h_rc[node1,node2]
            if time2 > l[node2]
                B_list2, fes= sequence_update(node2,time2,B_list2,prev2)
                time2 = B_list2[end]
            end
            sequence2 = [node2,cap2,time2,energy2,prev2,B_list2]
            push!(extended_route,sequence2)
        end
    end
    return extended_route
end
# eliminate sequence
function sequence_elimination(sequence_list)
    new_sequence_list= []
    for i in 1:length(sequence_list)
        sequence = sequence_list[i]
        node1,cap1,time1,energy1,prev,B_list = sequence[1:6]
        possible_dropoffs = [prev[i] + numb_user for i in 1:length(prev) if prev[i] in P_u && prev[i]+numb_user ∉ prev]
        dropoff_set = [] #store dropoff1, dropoff2, dropoff3
        # identify the dropoff that is farthest to the current vertex
        dis_dropoffs = Dict()
        if length(possible_dropoffs)>=1
            for j in possible_dropoffs
                dis_dropoffs[j] = d[node1,j]
            end
            dis_dropoffs = sort(collect(dis_dropoffs), by = tuple -> last(tuple), rev = true)
            dropoff1 = dis_dropoffs[1][1]
            push!(dropoff_set,dropoff1)
        end
        # identify the dropoff that is farthest to current vertex and dropoff1
        dis_dropoffs = Dict()
        if length(possible_dropoffs) >=2
            for j in possible_dropoffs
                dis_dropoffs[j] = d[node1,j] + d[dropoff1,j]
            end
            dis_dropoffs = sort(collect(dis_dropoffs), by = tuple -> last(tuple), rev = true)
            dropoff2 = dis_dropoffs[1][1]
            push!(dropoff_set,dropoff2)
        end
        # identify the dropoff that is farthest from dropoff1 and dropff2
        dis_dropoffs = Dict()
        if length(possible_dropoffs) >=3
            for j in possible_dropoffs
                dis_dropoffs[j] = d[dropoff1,j] + d[dropoff2,j]
            end
            dis_dropoffs = sort(collect(dis_dropoffs), by = tuple -> last(tuple), rev = true)
            dropoff3 = dis_dropoffs[1][1]
            push!(dropoff_set,dropoff3)
        end
        good_or_not = true
        for dropoff in dropoff_set
            if sequence_feasibility(sequence,dropoff) == false
                good_or_not = false
                break
            end
        end
        if good_or_not == true
            push!(new_sequence_list,sequence)
        end
    end
    return new_sequence_list
end
# input a pickup，output all the extensions
function all_sequences(node1,weight)
    finished_seq = []
    cap1 = q[node1]
    time1 = l[node1]
    #energy1 = min(minimum(h_rc[P_d[k],node1] for k in 1:numb_veh),minimum(h_rc[F[k],node1] for k in 1:length(charging_station)))
    energy1 = 0
    prev = [node1]
    B_list = [time1]
    sequence_init = [node1,cap1,time1,energy1,prev,B_list]
    unproceed_sequences = []
    new_sequences = sequence_extension(sequence_init)
    for i in 1:length(new_sequences)
        if new_sequences[i][2] == 0 #arrive at the next zero-load node
            sequence2 = new_sequences[i]
            node2,cap2,time2,energy2,prev2,B_list2 = sequence2[1:6]
            # 进行service begin time 优化
            B_list2,B_list2_prime = sequence_opt(prev2,B_list2)
            time2 = B_list2[end]
            ert = sequence_rt(prev2,B_list2)
            routing_cost2 = weight[1]*sum(t[prev2[i],prev2[i+1]] for i in 1:length(prev2)-1) + weight[2]*ert
            I_path2 = [node1,node2]
            I_path_tw2 = [[B_list2[1],B_list2_prime[1]],[B_list2[end],B_list2_prime[end]]]
            t_prime = B_list2[end]-B_list2[1]
            info_seq = [node2,cap2,time2,energy2,routing_cost2,prev2,B_list2,I_path2,I_path_tw2,t_prime,ert]
            push!(finished_seq,info_seq)
        else
            push!(unproceed_sequences,new_sequences[i])
        end
    end
    while unproceed_sequences != []
        sequence = unproceed_sequences[1]
        new_sequences = sequence_extension(sequence)
        new_sequences = sequence_elimination(new_sequences)
        for i in 1:length(new_sequences)
            if new_sequences[i][2] == 0 #arrive at the next zero-load node
                sequence2 = new_sequences[i]
                node2,cap2,time2,energy2,prev2,B_list2 = sequence2[1:6]
                # 进行service begin time 优化
                B_list2,B_list2_prime = sequence_opt(prev2,B_list2)
                time2 = B_list2[end]
                ert = sequence_rt(prev2,B_list2)
                routing_cost2 = weight[1]*sum(t[prev2[i],prev2[i+1]] for i in 1:length(prev2)-1) + weight[2]*ert
                I_path2 = [node1,node2]
                I_path_tw2 = [[B_list2[1],B_list2_prime[1]],[B_list2[end],B_list2_prime[end]]]
                t_prime = B_list2[end]-B_list2[1]
                info_seq = [node2,cap2,time2,energy2,routing_cost2,prev2,B_list2,I_path2,I_path_tw2,t_prime,ert]
                push!(finished_seq,info_seq)
            else
                push!(unproceed_sequences,new_sequences[i])
            end
        end
        deleteat!(unproceed_sequences,1)
    end
    return finished_seq
end
