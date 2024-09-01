# basic functions
function find_index(element,route)
  index = 0
  for i in 1:length(route)
    if element == route[i]
      index = i
    end
  end
  return index
end

function extract_request(solution,veh_numb)
    assigned_req = []
    for i in 1:length(solution[veh_numb])
        if solution[veh_numb][i] in P_u
            push!(assigned_req,solution[veh_numb][i])
        end
    end
    return assigned_req
end
# input route, output: the segments composed the route
function split_route(route)
  cap_list = [q[route[1]]]
  seg_list = []
  for i in 2:length(route)
    cap = cap_list[i-1] + q[route[i]]
    push!(cap_list,cap)
  end
  for i in 2:length(route)
    if cap_list[i] == q[route[i]] && route[i] ∈ P_u
      seg = [route[i]]
      for j in i+1:length(route)
        push!(seg,route[j])
        if cap_list[j] == 0 && route[j] ∈ D_u
          push!(seg_list,seg)
          break
        end
      end
    end
  end
  return seg_list
end

function simple_route(route)
  I_path = [route[1]]
  I_path_tw = [[e[route[1]],l[route[1]]]]
  rc_n1 = [i for i in route if i ∈ F]
  T_prime = []
  H_prime = []
  seg_list = split_route(route)
  if length(seg_list) > 0
    for i in 1:length(seg_list)
      ele1 = seg_list[i][1]
      for j in 1:length(sequences_list[ele1])
        sequence = sequences_list[ele1][j]
        if seg_list[i] == sequence[6]
          i_path = sequence[8]
          i_path_tw = sequence[9]
          t_prime = sequence[10] #已经包括了服务时间
          h_prime = sequence[4]
          I_path = vcat(I_path,i_path)
          push!(I_path_tw,i_path_tw[1])
          push!(I_path_tw,i_path_tw[2])
          push!(T_prime,t_prime)
          push!(H_prime,h_prime)
        end
      end
    end
  end
  push!(I_path,route[end])
  push!(I_path_tw,[e[route[end]],l[route[end]]])
  # 有充电站的话进行补充
  if length(rc_n1) > 0
    for i in 1:length(rc_n1)
      station = rc_n1[i]
      idx_rc = find_index(station,route)
      previous_node = route[idx_rc-1]
      idx_node = find_index(previous_node,I_path)
      idx_rc_prime = idx_node + 1
      insert!(I_path,idx_rc_prime,station)
      insert!(I_path_tw,idx_rc_prime,[e[station],l[station]])
    end
  end
  return I_path, I_path_tw, T_prime, H_prime
end

function slack_time_recharging(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime) #NOTE B_list(一定要是提前了过后的):lower bound of time window I-path
    slack = 0
    if node1 in charging_station
        slack = max(0,min(e_prime-min_time1-t_prime,max_rtime1))
    else
        slack = max(0,min(e_prime-min_time1-t_prime,max_time1-min_time1))
    end
    return slack
end
#X_{i,j}: the minimum recharging time that must be added at preceding recharging station to fulfill battery capacity constraint
function minimum_required_charging_time(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime,h_prime)
    slack = slack_time_recharging(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime)
    min_req_rt = max(0,max(0,max_rtime1-slack)+h_prime-H_rc)
    return min_req_rt
end
#####################
# Feasibility Check #
#####################
# vehicle load constraint check
function load_valid(route,k)
  feasibility = true
  seg_list = split_route(route)
  for i in 1:length(seg_list)
    segment = seg_list[i]
    if segment ∉ all_segments
      return false
    end
  end
  return feasibility
end

function battery_valid(route,k)
  route1, I_path_tw, T_prime, H_prime = simple_route(route)
  seg_list = split_route(route)
  rc_n1 = [i for i in route1 if i ∈ charging_station]
  # initial values for related resources
  T_min, T_max, T_rtmax = [0.0], [0.0], [0.0]
  feasibility = true

  counter = zeros(1,numb_recharging)
  if length(rc_n1) > 0
    for i in 1:length(rc_n1)
      station = rc_n1[i]
      idx_station = find_index(station,charging_station)
      counter[idx_station] += 1
      if counter[idx_station] >= 2
        return false
      end
    end
  end

  for i in 2:length(route1)
    node1 = route1[i-1]
    node2 = route1[i]
    if node1 ∈ P_u 
      idx_seg = find_index(node1,[seg_list[j][1] for j in 1:length(seg_list)])
      sequence = seg_list[idx_seg]
      t_prime = T_prime[idx_seg]
      h_prime = H_prime[idx_seg]
      e_prime = I_path_tw[i][1]
      l_prime = I_path_tw[i][2]
      # 计算T_min,T_max,T_rtmax
      min_time1 = T_min[i-1]
      max_time1 = T_max[i-1]
      max_rtime1 = T_rtmax[i-1]
      if length(rc_n1) == 0
        min_time2 = max(e_prime, min_time1 + t_prime)
        max_rtime2 = max_rtime1 + h_prime
      else
        min_time2 = max(e_prime, min_time1 + t_prime) + minimum_required_charging_time(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime,h_prime)
        max_rtime2 = min(H_rc,max(0,max_rtime1 -slack_time_recharging(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime))+h_prime)
      end
      max_time2 = min(l_prime,max(e_prime,max_time1+t_prime))
      push!(T_min,min_time2)
      push!(T_max,max_time2)
      push!(T_rtmax,max_rtime2)
      # judgement 1: T_j^{tmin} <= l_j
      if min_time2 <= l_prime
        feasibility = true
      else
        #println("from $node1 to $node2 the judgement 1 is violated!")
        return false
      end
      # judgement 2: T_j^{tmin} <= T_j^{tmax}
      if min_time2 <= max_time2
        feasibility = true
      else
        #println("from $node1 to $node2 the judgement 2 is violated!")
        return false
      end
      # judgement 3: T_j^{rtmax} <= H
      if max_rtime2 <= H_rc
        feasibility = true
      else
        #println("from $node1 to $node2 the judgement 3 is violated!")
        return false
      end
    else 
      t_prime = t[node1,node2] + s[node1]
      e_prime = e[node2]
      l_prime = l[node2]
      if node2 in D_d
        h_prime = h_rc[node1,node2] + H_rc*gama[1]
      else
        h_prime = h_rc[node1,node2]
      end
      # 计算T_min,T_max,T_rtmax
      min_time1 = T_min[i-1]
      max_time1 = T_max[i-1]
      max_rtime1 = T_rtmax[i-1]
      # calculate min_time2, max_time2, max_rtime2
      if length(rc_n1) == 0
        min_time2 = max(e_prime, min_time1 + t_prime)
        max_rtime2 = max_rtime1 + h_prime
      else
        min_time2 = max(e_prime, min_time1 + t_prime) + minimum_required_charging_time(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime,h_prime)
        max_rtime2 = min(H_rc,max(0,max_rtime1 -slack_time_recharging(node1, min_time1, max_time1, max_rtime1,node2,e_prime,t_prime))+h_prime)
      end
      if node1 in F
        max_time2 = min(l_prime,max(e_prime,min_time1+max_rtime1+t_prime))
      else
        max_time2 = min(l_prime,max(e_prime,max_time1+t_prime))
      end
      push!(T_min,min_time2)
      push!(T_max,max_time2)
      push!(T_rtmax,max_rtime2)
      # judgement 1:
      if min_time2 <= l_prime
        feasibility = true
      else
        #println("from $node1 to $node2 the judgement 1 is violated!")
        return false
      end
      # judgement 2:
      if min_time2 <= max_time2
        feasibility = true
      else
        #println("from $node1 to $node2 the judgement 2 is violated!")
        return false
      end
      # judgement 3:
      if node2 in D_d
        if max_rtime2 <= H_rc
          feasibility = true
        else
          #println("from $node1 to $node2 the judgement 3 is violated!")
          return false
        end
      else
        if max_rtime2 <= H_rc
          feasibility = true
        else
          #println("from $node1 to $node2 the judgement 3 is violated!")
          return false
        end
      end
    end
  end
  return feasibility
end

function is_feasible_tour(route,k)
  feasibility = 1

  if load_valid(route,k)&& battery_valid(route,k)[1]
    feasibility = 1
  else
    feasibility = 0
  end
  return feasibility == 1
end

function compute_cost(route,k,weight)
  travel_time = sum(t[route[i],route[i+1]] for i in 1:length(route)-1)
  excess_ride_time = 0.0
  seg_cost = []
  seg_list = split_route(route)
  if length(seg_list) > 0
    for i in 1:length(seg_list)
      ele1 = seg_list[i][1]
      for j in 1:length(sequences_list[ele1])
        sequence = sequences_list[ele1][j]
        if seg_list[i] == sequence[6]
          ert_cost = sequence[11]
          excess_ride_time += ert_cost
        end
      end
    end
  end
  total_cost = travel_time*weight[1] + excess_ride_time*weight[2]
  return total_cost, travel_time, excess_ride_time
end
