# Constructive algorithm for Dial-a-ride problem with electric vehicles considering
# different driving styles
# NOTE: if we have more destination to choose, we should randomly select rather than assigned D_d[k] as in algorithm
# select the best vehicle (min distance) to insert new request
function Insertion_cost(i,j,k) #insert k into (i,j)
  cost = t[i,k]+ t[k,j] - t[i,j]
  return cost
end

function Insertion(initial_route,veh_num,element)
  cost_insertion = Dict()
  # starting from the beginning of vehicle's route until the end
  if element in P_u # insert pickup node
    for i in 2:length(initial_route[veh_num])
      #route = insert!(initial_route[veh_num],i,element)
      ele1 = initial_route[veh_num][i-1]
      ele2 = initial_route[veh_num][i]
      cost_insertion[element,i] = Insertion_cost(ele1,ele2,element)
      #deleteat!(initial_route[veh_num],i) # recover the route
    end
  else #insert dropoff node
    # find the position of corresponding pickup node
    pos = find_index(element-numb_user,initial_route[veh_num])
    if pos != length(initial_route[veh_num]) #corresponding pickup is not at the end
      for i in pos+1:length(initial_route[veh_num]) # from pos+1 visit all potential position
        #route = insert!(initial_route[veh_num],i,element)
        ele1 = initial_route[veh_num][i-1]
        ele2 = initial_route[veh_num][i]
        cost_insertion[element,i] = Insertion_cost(ele1,ele2,element)
      end
    else # corresponding pickup at the end of route
      cost_insertion[element,length(initial_route[veh_num])+1] = t[initial_route[veh_num][end],element]
    end
  end
  return cost_insertion
end


function ConsHeuristic(weight)
  function SelectVehicle(new_element,assigned_users)
    sorted_time = Dict()
    for k in veh_assigned
      last_assigned = assigned_users[k][end]
      # compute the distance between first non-assigned user and last assigned user of initial route
      sorted_time[k] = t[last_assigned,new_element]
    end
    sorted_time = sort(collect(sorted_time), by = tuple -> last(tuple))
    return sorted_time # return vehicle list sorted with travel time
  end

  ###################################
  # Parallel construction algorithm #
  ###################################
  # STEP 1: initialize the set of non-assigned users sorted in increasing order
  # of earliest pickup time
  Earliest_Pickup = Dict()
  for i in P_u
    Earliest_Pickup[[i,numb_user+i]] = e[i]
  end
  sorted_Pickup = sort(collect(Earliest_Pickup), by = tuple -> last(tuple))
  # randomly select m1 vehicles and assign the first m1 users to different vehicles
  #Random.seed!(1234); # set a fix random seed
  m1 = rand(K)

  #randomly assign the first m1 requests (ealiest pickup time) to the m1 avaliable vehicles, one request per vehicle
  rand_veh = []
  veh = [k for k in K]
  for k in 1:m1
      push!(rand_veh,rand(veh))
      deleteat!(veh,find_index(rand_veh[k],veh)) # the rest element in veh is non_assign vehicles
  end

  # construct initially routes for random selected vehicles
  initial_route = Dict()
  for i in rand_veh
    for j in 1:length(sorted_Pickup)
      if is_feasible_tour([P_d[i],sorted_Pickup[j][1][1],sorted_Pickup[j][1][2],D_d[i]],i)
        initial_route[i] = sorted_Pickup[j][1]
        deleteat!(sorted_Pickup,j)
        break
      end
    end
  end

  veh_assigned = rand_veh
  # record assigned requests
  assigned_users = Dict()
  for k in veh_assigned
    assigned_users[k] = [initial_route[k][1]]
    pushfirst!(initial_route[k],P_d[k])
    push!(initial_route[k],D_d[k])
  end
  ###############################
  # Iteratively construct route #
  ###############################
  index4 = []
  iter = 1
  # inserting users one by one into the route while respecting the constraint
  # first: sorted routes by distance between the last assigned user in this route and the first user in the list
  # second: check the feasibility of pickup and corresponding drop-off node
  # if no vehicle in the veh_assigned list can accept this request then activate a new route
  while length(sorted_Pickup) != 0
    non_assigned_request1 = length(sorted_Pickup)
    i = 1
    while i <= length(sorted_Pickup) #check every request in the sorted list
      #global i
      judge = 1
      while judge <= length(veh_assigned) # check every assigned vehicle
        # new request to be inserted
        if i > length(sorted_Pickup)
          break
        end
        pickup = sorted_Pickup[i][1][1]
        dropoff = sorted_Pickup[i][1][2]
        # select the best vehicle (minimum travel time) to insert the request
        veh_numb = SelectVehicle(pickup,assigned_users)[judge][1]
        # insert pickup and dropoff nodes in the best position
        cost_insertion1 = Insertion(initial_route,veh_numb,pickup)
        cost_insertion1 = sort(collect(cost_insertion1), by = tuple -> last(tuple))
        while length(cost_insertion1) > 0 # check all the potential position to insert
          insert!(initial_route[veh_numb],cost_insertion1[1][1][2],cost_insertion1[1][1][1])
          cost_insertion2 = Insertion(initial_route,veh_numb,dropoff)
          cost_insertion2 = sort(collect(cost_insertion2), by = tuple -> last(tuple))
          insert!(initial_route[veh_numb],cost_insertion2[1][1][2],cost_insertion2[1][1][1])
          # feasibility checking
          if is_feasible_tour(initial_route[veh_numb],veh_numb)
            push!(assigned_users[veh_numb],pickup)
            deleteat!(sorted_Pickup,i) #delete newly inserted request
            judge = 1
            break # 代表已经把这个request插入到了最好的位置，跳出最内while循环 (插入路径操作结束)
          else #NOTE 判断是不是违反了电量约束
            deleteat!(initial_route[veh_numb], cost_insertion2[1][1][2])
            deleteat!(initial_route[veh_numb], cost_insertion1[1][1][2])
            deleteat!(cost_insertion1,1)
          end
        end
        if length(cost_insertion1) == 0 # 说明该车的路线不能插入这个request
          judge +=1
        end
      end# 第三层while循环结束，关于该request所有车中可以插入的位置检查完毕
      i+=1
    end # 第二层while循环结束，关于所有requests检查完毕
    if veh != [] && length(sorted_Pickup) > 0 # activate new vehicle
      for j in 1:length(sorted_Pickup)
        if is_feasible_tour([P_d[veh[1]],sorted_Pickup[j][1][1],sorted_Pickup[j][1][2],D_d[veh[1]]],veh[1])
          push!(initial_route, veh[1]=>[P_d[veh[1]],sorted_Pickup[j][1][1],sorted_Pickup[j][1][2],D_d[veh[1]]])
          push!(veh_assigned, veh[1])
          push!(assigned_users, veh[1]=> [sorted_Pickup[j][1][1]])
          deleteat!(veh,1)
          push!(index4,j) # index4 records the inserted request index in sorted_Pickup
          break #一次只插入一辆新车
        end
      end
    end
    if index4 !=[] # 存在插入到新route里面的request
      deleteat!(sorted_Pickup, index4[1])
      deleteat!(index4,1)
    end
    non_assigned_request2 = length(sorted_Pickup)
    if non_assigned_request1 == non_assigned_request2 #没有能够插入route中的request了
      break
    end
  end
  # 如果最后还有车辆没有安排，则该路径为[P_d[i],D_d[i]]
  if veh != []
    for i in 1:length(veh)
      initial_route[veh[i]] = [P_d[veh[i]], D_d[veh[i]]]
    end
  end

  initial_solution = deepcopy(initial_route)

  # print results
  numb_request = sum(length(assigned_users[k]) for k in veh_assigned)
  total_cost = sum(compute_cost(initial_route[k],k,weight)[1] for k in rand_veh)
  travel_cost = sum(compute_cost(initial_route[k],k,weight)[2] for k in rand_veh)
  excess_ride_cost = sum(compute_cost(initial_route[k],k,weight)[3] for k in rand_veh)
  println("dataset:",numb_veh,"-",numb_user)
  println("parallel insertion algorithm 1")
  println("the number of assigned requests is:", numb_request)
  println("the objective function value is:", total_cost)
  println("the travel cost is:", travel_cost)
  println("the excess ride time is:", excess_ride_cost)

  return initial_solution
end
