"""
 Filters for columns
"""
# calculate the set of lower bound with branches
function filter_col_with_flow_var(cols,cost_cols,flow_branch_decision)
    columns = []
    cost_columns = []
    # if branch_decision has branch on flow var, filter inherited columns
    if flow_branch_decision != []
        branch_type, branch_var, branch_direction = flow_branch_decision[1:3]
        if branch_type == "flow_var" && branch_direction == "<=" # forbiden arc from i to j
            println("the flow variable decision is:", flow_branch_decision)
            first_node = branch_var[1]
            second_node = branch_var[2]
            for j in 1:length(cols)
                column = cols[j]
                cost_column = cost_cols[j]
                if first_node ∉ column || second_node ∉ column
                    push!(columns,column)
                    push!(cost_columns,cost_column)
                else
                    idx1 = find_index(first_node,column)
                    idx2 = find_index(second_node,column)
                    if idx1 + 1 != idx2
                        push!(columns,column)
                        push!(cost_columns,cost_column)
                    end
                end
            end
        elseif branch_type == "flow_var" && branch_direction == ">="
            println("the flow variable decision is:", flow_branch_decision)
            first_node = branch_var[1]
            second_node = branch_var[2]
            for j in 1:length(cols)
                column = cols[j]
                cost_column = cost_cols[j]
                if first_node in column
                    idx1 = find_index(first_node,column)
                    idx2 = idx1+1
                    if column[idx2] == second_node
                        push!(columns,column)
                        push!(cost_columns,cost_column)
                    end
                end
                if second_node in column
                    idx2 = find_index(second_node,column)
                    idx1 = idx2-1
                    if idx1 != 0 && column[idx1] == first_node
                        push!(columns,column)
                        push!(cost_columns,cost_column)
                    end
                end
                if first_node ∉ column && second_node ∉ column
                    push!(columns,column)
                    push!(cost_columns,cost_column)
                end
            end
        else
            #decision_counter += 1
            columns = deepcopy(cols)
            cost_columns = deepcopy(cost_cols)
        end
    else
        columns = deepcopy(cols)
        cost_columns = deepcopy(cost_cols)
    end
    return columns, cost_columns
end

function filter_columns(cols,cost_cols,branch_decision)
    filtered_cols, filtered_cost_cols = deepcopy(cols), deepcopy(cost_cols)
    if branch_decision != []
        for i in 1:length(branch_decision)
            decision = branch_decision[i]
            branch_type, branch_var, branch_direction = decision[1:3]
            if branch_type == "flow_var"
                filtered_cols, filtered_cost_cols = filter_col_with_flow_var(filtered_cols, filtered_cost_cols,decision)
            end
        end
    else
        return cols, cost_cols
    end
    return filtered_cols, filtered_cost_cols
end
