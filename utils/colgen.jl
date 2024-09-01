##################
# Master problem #
##################
function get_master(omega, cost_omega, weight, branch_decision)
    b1 = zeros(numb_nodes, length(omega))
    b2 = zeros(numb_nodes, length(omega))
    for i in 1:length(omega)
        for j in 1:length(omega[i])
            if omega[i][j] in P_u
                b1[omega[i][j],i] += 1
            end
            if omega[i][j] in union(F,D_d)
                b2[omega[i][j],i] +=1
            end
        end
    end
    # cost matrix
    c = deepcopy(cost_omega)
    Pen = ones(numb_user)*1e9 # penalization
    # 定义 set covering problem
    SCP = Model(Gurobi.Optimizer)
    set_optimizer_attribute(SCP,"TimeLimit",600)
    @variable(SCP, y[w in 1:length(omega)]) # y_w = 1, the route is used in the solution
    @variable(SCP, a[i in P_u]) # a_i = 1 if request i is not severed in the solution
    @objective(SCP, Min, sum((c[w][1]*weight[1]+c[w][2]*weight[2])*y[w] for w in 1:length(omega)) + sum(Pen[i]*a[i] for i in P_u))
    # constraints
    cons1 = Dict()
    for i in P_u
        cons1[i] = @constraint(SCP, sum(b1[i,w]*y[w] for w in 1:length(omega)) >= 1 - a[i])
    end
    cons2 = Dict()
    for i in union(F,D_d) # for each i in F can be only visited once
        cons2[i] = @constraint(SCP, sum(b2[i,w]*y[w] for w in 1:length(omega)) <= 1)
    end
    @constraint(SCP, cons3, sum(y[w] for w in 1:length(omega)) <= numb_veh)

    # NOTE: 利用var 以及veh_cond 添加新的约束条件:注意子节点要继承父节点的constraint
    new_cons = Dict()
    #首先要排除"flow_var"
    if branch_decision != []
        for i in 1:length(branch_decision)
            # 利用字符串来定义分支策略
            branch_type, branch_var, branch_direction = branch_decision[i][1:3]
            if branch_direction == ">="
                if branch_type == "total_veh"
                    new_cons[i] = @constraint(SCP, sum(y[w] for w in 1:length(omega)) >= branch_var)
                else
                    new_cons[i] = []
                end
            elseif branch_direction == "<="
                if branch_type == "total_veh"
                    new_cons[i] = @constraint(SCP, sum(y[w] for w in 1:length(omega)) <= branch_var)
                else
                    new_cons[i] = []
                end
            end
        end
    end
    # other constraints
    for w in 1:length(omega)
        @constraint(SCP,y[w] >= 0)
    end
    for i in P_u
      @constraint(SCP,a[i] >= 0)
    end
    JuMP.optimize!(SCP)
    # 对status进行判断
    primal_feasible = primal_status(SCP) == JuMP.MOI.FEASIBLE_POINT
    dual_feasible = dual_status(SCP) == JuMP.MOI.FEASIBLE_POINT
    is_rmp_optimal = primal_feasible &&  dual_feasible

    if is_rmp_optimal == true
        obj = JuMP.objective_value(SCP)
        println("the objective function value is = $obj")
        ###################################### retrive dual information #################################
        println("############### the price of each job #######################")
        dual1 = zeros(length(cons1))
        for i in 1:length(cons1)
            dual1[i] = dual(cons1[i])
            println(dual1[i])
        end
        println("############### the price of each stations #######################")
        dual2 = zeros(length(cons2))
        for i in 1:length(cons2)
            dual2[i] = dual(cons2[i+P_d[numb_veh]])
            println(dual2[i])
        end
        println("############## the price of branching and constant constraints##########")
        dual3 = dual(cons3)
        # for newly-added constraints, retrieve the dual variables
        add_dual = []
        if new_cons != Dict()
            for i in 1:length(new_cons)
                if new_cons[i] != []
                    new_dual = dual(new_cons[i])
                    push!(add_dual,new_dual)
                end
            end
        end
        # 把newly-introduced的constraints的对偶变量值合并到dual3里面
        dual3_prime = dual3
        if add_dual != []
            dual3_prime += sum(add_dual)
        end
        println(dual3_prime)
        println("#################################################################################")
        # return routes (solution) that used in the basis
        solution = []
        frac = []
        for i in 1:length(omega)
          if JuMP.value.(y[i]) > 0
            println(JuMP.value.(y[i]))
            push!(solution, omega[i])
            push!(frac,JuMP.value.(y[i]))
          end
        end
        return dual1, dual2, dual3_prime, solution, obj, frac, is_rmp_optimal
    else
        return [], [], 0, [], Inf, [], false
    end
end

# upper bound calculation (integer RMP)
function Int_RMP(omega,cost_omega,weight,branch_decision)
    # 定义系数矩阵
    b1 = zeros(numb_nodes, length(omega))
    b2 = zeros(numb_nodes, length(omega))
    for i in 1:length(omega)
        for j in 1:length(omega[i])
            if omega[i][j] in P_u
                b1[omega[i][j],i] += 1
            end
            if omega[i][j] in union(F,D_d)
                b2[omega[i][j],i] +=1
            end
        end
    end

    # cost matrix
    #c = zeros(length(omega))
    c = deepcopy(cost_omega)
    Pen = ones(numb_user)*1000 # penalization

    # 定义 set covering problem
    SCP = Model(Gurobi.Optimizer)
    set_optimizer_attribute(SCP,"TimeLimit",3600)
    @variable(SCP, y[w in 1:length(omega)], Bin) # y_w = 1, the route is used in the solution
    @variable(SCP, a[i in P_u], Bin)
    @objective(SCP, Min, sum((c[w][1]*weight[1]+c[w][2]*weight[2])*y[w] for w in 1:length(omega)) + sum(Pen[i]*a[i] for i in P_u))
    # constraints
    cons1 = Dict()
    for i in P_u
        cons1[i] = @constraint(SCP, sum(b1[i,w]*y[w] for w in 1:length(omega)) >= 1 - a[i])
    end
    cons2 = Dict()
    for i in union(F,D_d) # for each i in F can be only visited once
        cons2[i] = @constraint(SCP, sum(b2[i,w]*y[w] for w in 1:length(omega)) <= 1)
    end
    @constraint(SCP, cons3, sum(y[w] for w in 1:length(omega)) <= numb_veh)

    JuMP.optimize!(SCP)

    is_mip_optimal = has_values(SCP)

    if is_mip_optimal == true
        obj2 = JuMP.objective_value(SCP)
        solution2 = []
        for i in 1:length(omega)
          if JuMP.value.(y[i]) > 0.99
            println(JuMP.value.(y[i]))
            push!(solution2, omega[i])
          end
        end
        return obj2, solution2
    else
        return Inf, []
    end
end

function column_generation(omega,cost_omega,branch_decision,weight)
    column_pool = deepcopy(omega)
    cost_columns = deepcopy(cost_omega)
    reduced_cost_list = []
    #upper_bound = []
    lower_bound = []
    continuous_RMP = []
    duals = []
    dual1, dual2, dual3, solution, obj1, frac, is_rmp_optimal = get_master(column_pool,cost_columns,weight,branch_decision)
    if !is_rmp_optimal
        return Inf, Inf, [], [], column_pool, cost_columns
    end
    push!(duals,dual1)
    push!(duals,dual2)
    push!(duals,dual3)
    push!(continuous_RMP,obj1)

    iter = 0
    elapsedTime = 0
    obj = -1000
    t1 = time_ns()
    # terminate criterion: no more negative reduced cost column, reach iteration times, reach time limit (5 hours)
    while elapsedTime <= 3600*2 && obj < -0.0001
        t2 = time_ns()
        elapsedTime = (t2-t1)/1.0e9
        println(dual1)
        println(dual2)
        println(dual3)
        # cost_column: the actual routing cost of each column
        # best_label, new_column, cost_column = labeling_algorithm(duals,weight,branch_decision)
        # obj = best_label[2][1]
        obj, new_column, cost_column = column_generator(duals,weight,branch_decision)
        #println(new_column)
        println("\e[1;45m the best objective from labeling algorithm is: $obj \e[00m")
        push!(lower_bound, obj1+numb_veh*obj)
        if length(new_column) > 0
            for i in 1:length(new_column)
                push!(column_pool, new_column[i])
                push!(cost_columns,cost_column[i])
            end
        end
        # resolve master problem
        dual1, dual2, dual3, solution, obj1, frac, is_rmp_optimal = get_master(column_pool,cost_columns,weight,branch_decision)
        if !is_rmp_optimal
          return Inf, Inf, [], [], column_pool, cost_columns
        end
        duals = []
        push!(duals,dual1)
        push!(duals,dual2)
        push!(duals,dual3)
        push!(continuous_RMP, obj1)
        # update iteration times
        iter += 1
    end
    #println("\e[1;45m the continuous RMP objective value is [$(obj1)] \e[00m")
    # 如果最后的解仍然cost>1000 (infeasible), 则返回[]
    if obj1 > 1000
        return Inf, Inf, [], [], column_pool, cost_columns
    else
        return lower_bound, continuous_RMP, solution, frac, column_pool, cost_columns
    end
end
