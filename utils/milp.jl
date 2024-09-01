"""
    Example: using a MILP solver + callback to find multiple negative reduced cost columns:
    Input: dual variable values
    Output: feasible columns with negative reduced cost
"""
function column_generator(duals,weight,branch_decision)
    dual1, dual2, dual3 = duals[1], duals[2], duals[3]
    e_ADARP3 = Model(Gurobi.Optimizer)
    set_optimizer_attribute(e_ADARP3,"TimeLimit",3600)
    @variable(e_ADARP3,x[i in V,j in V], Bin)
    @variable(e_ADARP3,T[i in V]) # the time at which vehicle k begins service at location i
    @variable(e_ADARP3,L[i in V]) # load of vehicle k after service
    @variable(e_ADARP3,R[i in P_u]) # excess user ride time
    # model construction
    exp1 = @expression(e_ADARP3, sum(dual1[i]*x[i,j] for i in P_u, j in setdiff(V,P_d)))
    exp2 = @expression(e_ADARP3, sum(dual2[i-P_d[numb_veh]]*x[i,j] for i in union(F,D_d), j in union(D_u,F,P_d)))
    y1 = @expression(e_ADARP3, weight[1]*sum(x[i,j]*t[i,j] for i in V, j in setdiff(V,P_d)) + weight[2]*sum(R[i] for i in P_u) - exp1 - exp2 -dual3)
    @objective(e_ADARP3, Min, y1)
    # infeasible arcs elimination
    @constraint(e_ADARP3,sum(x[i,j] for i in union(P_d,F), j in D_u) == 0) # no passenger abord at recharging station
    @constraint(e_ADARP3,sum(x[i,j] for i in P_u, j in union(D_d,F)) == 0) # infeasible arc (pickup, destination/recharging)
    @constraint(e_ADARP3,sum(x[i,j] for i in setdiff(V,P_d), j in P_d) == 0) # infeasible arc (other nodes, origin)
    @constraint(e_ADARP3,sum(x[i,j] for i in D_d, j in setdiff(V,D_d)) == 0)
    @constraint(e_ADARP3,sum(x[i,i] for i in V)==0)
    @constraint(e_ADARP3,sum(x[numb_user+i,i] for i in P_u)==0)

    # consider the branching decision on flow
    if branch_decision != []
        for i in 1:length(branch_decision)
            branch_type, branch_var, branch_direction = branch_decision[i][1:3]
            if branch_type == "flow_var" && branch_direction == "<=" # from i cannot go to j
                first_node = branch_var[1]
                second_node = branch_var[2]
                @constraint(e_ADARP3,x[first_node,second_node] == 0)
            elseif branch_type == "flow_var" && branch_direction == ">=" # from i must go to j
                first_node = branch_var[1]
                second_node = branch_var[2]
                @constraint(e_ADARP3,x[first_node,second_node] == 1)
            end
        end
    end

    # subject to:

    @constraint(e_ADARP3,sum(x[i,j] for i in P_d, j in union(P_u,F,D_d)) == 1)

    @constraint(e_ADARP3,sum(x[i,j] for j in D_d, i in union(D_u,F,P_d)) == 1)

    for i in union(N,F)
        @constraint(e_ADARP3,sum(x[i,j] for j in setdiff(V,P_d) if j!=i) - sum(x[j,i] for j in setdiff(V,D_d) if j!=i) == 0)
    end


    for i in P_u
        @constraint(e_ADARP3, sum(x[i,j] for j in N if j !=i) - sum(x[j,numb_user+i] for j in N if j!= numb_user+i) == 0)
    end

    for i in P_u
        @constraint(e_ADARP3,T[i] + s[i] + t[i,numb_user+i] <= T[numb_user+i])
    end

    # time window constraints
    for i in V
        @constraint(e_ADARP3, e[i] <= T[i])
        @constraint(e_ADARP3, T[i] <= l[i])
    end

    # maximum user ride time constraints
    for i in P_u
        @constraint(e_ADARP3, T[numb_user+i] - T[i] - s[i] <= m[i])
    end

    for i in V
        for j in setdiff(V,P_d)
            @constraint(e_ADARP3, T[i] + t[i,j] +s[i] - l[D_d[1]]*(1-x[i,j]) <= T[j])
        end
    end

    for i in P_u
        @constraint(e_ADARP3, R[i] >= T[numb_user+i] - T[i] - s[i] - t[i,numb_user+i])
    end

    for i in V
        for j in V
                @constraint(e_ADARP3, L[i] + q[j] - (VC[1]+1)*(1-x[i,j]) <= L[j])
                @constraint(e_ADARP3, L[i] + q[j] + (VC[1]+1)*(1-x[i,j]) >= L[j])
        end
    end

    for i in N
        @constraint(e_ADARP3,L[i] >= max(0,q[i]))
        @constraint(e_ADARP3,L[i] <= min(VC[1],VC[1]+q[i]))
    end

    for i in union(P_d,D_d,F)
        @constraint(e_ADARP3, L[i] ==0)
    end

    for i in V
        # @constraint(e_ADARP3, B[i] >= 0)
        @constraint(e_ADARP3, T[i] >= 0)
    end

    # for f in F
    #     @constraint(e_ADARP3, E[f] >= 0)
    # end

    for i in V
        @constraint(e_ADARP3, T[i] >= 0)
    end

    # callback function to generate multiple columns while solving the MIP
    column_pool = []
    cb_calls = Cint[]
    function my_callback_function(cb_data, cb_where::Cint)
        # You can reference variables outside the function as normal
        push!(cb_calls, cb_where)
        # You can select where the callback is run: when found a MIP incumbent
        if cb_where != GRB_CB_MIPSOL
            return
        end
        # Before querying `callback_value`, you must call:
        fes_sol = []
        Gurobi.load_callback_variable_primal(cb_data, cb_where)
        y_val = callback_value(cb_data, y1)
        if y_val < 0.000001
            for i in V
                for j in V
                    x_val = callback_value(cb_data, x[i,j])
                    if x_val > 0.99
                        push!(fes_sol,[i,j])
                    end
                end
            end
        end
        #println(fes_sol)
        push!(column_pool, fes_sol)
    end

    MOI.set(e_ADARP3, Gurobi.CallbackFunction(), my_callback_function)

    optimize!(e_ADARP3)

    obj = JuMP.objective_value(e_ADARP3)

    """
        post-preprocessing: arcs -> columns
    """
    columns = []
    for sol in 1:length(column_pool)
        if length(column_pool[sol]) != 0
            route_set = []
            station = []
            for j in 1:length(column_pool[sol])
                if column_pool[sol][j][1] in P_d
                    push!(station, column_pool[sol][j])
                end
            end
            for j1 in 1:length(column_pool[sol])
                for j2 in 1:length(column_pool[sol])
                    if station[j1][2] == column_pool[sol][j2][1]
                        push!(station,column_pool[sol][j2])
                    end
                end
            end
            new_route = [station[1][1], station[1][2]]
            for j in 2:length(column_pool[sol])
                push!(new_route,station[j][2])
            end
            push!(columns, new_route)
        end
    end
    cost_columns = []
    for i in 1:length(columns)
        first_obj, second_obj = compute_cost(columns[i],1,weight)[2],compute_cost(columns[i],1,weight)[3]
        push!(cost_columns,[first_obj,second_obj])
    end
    return obj, columns, cost_columns
end
