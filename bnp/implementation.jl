"""
 Problem specific implementation of BnP
"""
#################################################
# Root node calculation and tree initialization #
#################################################
function root_calculate(cols, cost_cols, weight, branch_decision)
    if cols == []
        initial_solution = ConsHeuristic(weight)
        for k in 1:length(initial_solution)
            push!(cols,initial_solution[k])
            travel_time, ert_cost = compute_cost(initial_solution[k],k,weight)[2:3]
            push!(cost_cols,[travel_time,ert_cost])
        end
    end
    columns, cost_columns = filter_columns(cols,cost_cols,branch_decision)
    lb, continuous_RMP, frac_sol, frac, columns, cost_columns = column_generation(columns, cost_columns, branch_decision, weight)
    if lb != Inf
        return columns, cost_columns, frac_sol, frac, lb[end]
    else
        return columns, cost_columns, frac_sol, frac, Inf
    end
end

function calculation(node::BranchAndPrice.JuMPNode,weight)
    cols = node.cols
    cost_cols = node.cost_cols
    branch_decision = node.branch
    columns, cost_columns = filter_columns(cols,cost_cols,branch_decision)
    # input filtered columns and cost_columns to CG
    lb, continuous_RMP, frac_sol, frac, columns, cost_columns = column_generation(columns,cost_columns,branch_decision,weight)
    if lb != Inf
        return columns, cost_columns, frac_sol, frac, lb[end] #,best_incumbent, best_sol
    else
        return columns, cost_columns, frac_sol, frac, Inf #,best_incumbent, best_sol
    end
end

# basic bounding function: calculation at each node
function bound!(node::BranchAndPrice.JuMPNode,weight)

    cols, cost_cols, frac_sol, frac, bound = calculation(node,weight)

    if bound < Inf
        node.bound = bound
        node.solution = deepcopy(frac_sol)
        node.fraction = deepcopy(frac)
        node.cols = deepcopy(cols)
        node.cost_cols = deepcopy(cost_cols)
    else
        node.bound = Inf
    end
end

function branch!(tree::BranchAndPrice.AbstractTree, node::BranchAndPrice.AbstractNode)
    println("\e[1;45m Node id $(node.id) , bound $(node.bound), best incumbent $(tree.best_incumbent)\e[00m")
    if node.bound >= tree.best_incumbent #tree.best_incumbent是upper bound
        @info " Fathomed by bound"
    elseif node.bound < Inf # the node is solvable
        # 确定分支策略
        branches1, branches2 = deepcopy(node.branch), deepcopy(node.branch)
        branch_info = branching_decision(node.solution,node.fraction)
        if branch_info != []
            if branch_info[1] == "total_veh" #|| branch_info[1] == "total_rec"
                branch_decision1 = [branch_info[1], floor(branch_info[2]) ,"<="]
                branch_decision2 = [branch_info[1], ceil(branch_info[2]) ,">="]
            else
                branch_decision1 = [branch_info[1], branch_info[2] ,"<="]
                branch_decision2 = [branch_info[1], branch_info[2] ,">="]
            end
            push!(branches1, branch_decision1)
            push!(branches2, branch_decision2)
            if branch_info[1] == "total_veh"
                @info " Branch at $(branch_info[1]), x >= $(ceil(branch_info[2])), x <= $(floor(branch_info[2]))"
            else
                @info " Branch at $(branch_info[1]) on variable x$(branch_info[2]), x >= 1, x <= 0"
            end
            # create childs
            child1 = BranchAndPrice.create_child_node(node, branches1) # 除了solution,以及fraction,其余都继承parent
            child2 = BranchAndPrice.create_child_node(node, branches2)
            push!(tree, child1)
            push!(tree, child2)
        else #如果该节点是整数解
            @info " Fathomed by integrality"
            if node.bound < tree.best_incumbent
                # 更新best_incumbent
                @info " New incumbent bound: $(node.bound)"
                tree.best_incumbent = node.bound
            end
        end
    else # node.bound == Inf # the node is unsolvable
        @info " Fathomed by solution status: INFEASIBLE"
    end
end


function run(tree::BranchAndPrice.AbstractTree,int_sol,weight)
    elapsedTime = 0
    t1 = time_ns()
    best_int_sol = deepcopy(int_sol)
    best_int_cost = tree.best_incumbent
    total_nodes = 0
    remain_nodes = 0
    # 如果在根结点处就已经收敛，则直接把node.solution赋给best_int_sol
    if integer_or_not(tree.nodes[1].fraction) == true
        best_int_sol = deepcopy(tree.nodes[1].solution)
    end
    while !BranchAndPrice.termination(tree) && elapsedTime <=3600*2
        node = BranchAndPrice.next_node(tree) #返回tree底端的node
        BranchAndPrice.update_best_bound!(tree) #返回tree.nodes中所有node的bound的最小值

        bound!(node,weight) #用column generation计算
        #将整数解存下来
        if integer_or_not(node.fraction) == true
            if node.bound < best_int_cost
                best_int_sol = deepcopy(node.solution)
                best_int_cost = node.bound
            end
        end
        BranchAndPrice.processed!(tree, node)
        branch!(tree, node) # 根据分支策略，生成两个abstract node, 此时tree.nodes更新为生成的两个子节点
        t2 = time_ns()
        elapsedTime = (t2-t1)/1.0e9
        if elapsedTime > 3600*2
            @info "Reached maximum time limit"
        end
    end
    total_nodes = length(tree.processed)
    remain_nodes = length(tree.nodes)
    return best_int_sol, best_int_cost, total_nodes, remain_nodes
end

###############
# Run program #
###############
function run_BP(cols, cost_cols, weight, branch_decision)
    # calculate the root node results
    time0 = @elapsed cols, cost_cols, frac_sol, frac, bound = root_calculate(cols, cost_cols,weight,branch_decision)
    tree = BranchAndPrice.initialize_tree(cols, cost_cols, frac_sol, frac, bound)
    root = tree.nodes[1]
    #NOTE: 继承 branch_decision
    root.branch = deepcopy(branch_decision)
    # calculate the upper bound with columns generated
    best_incumbent, best_sol = Int_RMP(cols,cost_cols,weight,branch_decision)
    # update best_bound, best_incumbent of the tree
    tree.best_bound = root.bound
    tree.best_incumbent = best_incumbent
    # apply B&P
    time1 = @elapsed best_int_sol, best_int_cost, total_nodes, remain_nodes = run(tree,best_sol,weight)
    total_time = time0 + time1
    cols, cost_cols = root.cols, root.cost_cols
    return best_int_cost, best_int_sol, total_time, cols, cost_cols, total_nodes, remain_nodes
end
