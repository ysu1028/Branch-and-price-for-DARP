
function branching_decision(frac_sol,fraction)
    #frac = round.(fraction,digits=4)
    frac = deepcopy(fraction)
    if integer_or_not(fraction) == true
        return []
    end
    # branch on the total number of routes
    if round.(sum(fraction),digits=2)%1 != 0.0  
        return ["total_veh", sum(frac)] 
    end
    # branch on the total flow on an arc of G
    arc_list = []
    arc_frac_list = []
    for i in 1:length(frac_sol)
        for j in 1:length(frac_sol[i])-1
            ele1 = frac_sol[i][j]
            ele2 = frac_sol[i][j+1]
            push!(arc_list,[ele1,ele2])
            push!(arc_frac_list,frac[i])
        end
    end
    frac_variable = []
    fractions = []
    for i in 1:length(arc_list)-1
        cur_arc = arc_list[i]
        values = arc_frac_list[i]
        if cur_arc ∉ frac_variable
            for j in i+1:length(arc_list)
                if arc_list[j] == cur_arc
                    values += arc_frac_list[j]
                end
            end
            push!(frac_variable,cur_arc)
            push!(fractions,values)
        end
    end
    
    frac_variable1 = []
    fractions1 = []
    for i in 1:length(fractions)
        #if floor(fractions[i]) != round.(fractions[i],digits=4) && frac_variable[i][1] ∉ P_d # NOTE
        if round.(fractions[i],digits=4)%1 != 0.0 && frac_variable[i][1] ∉ P_d
            push!(frac_variable1,frac_variable[i])
            push!(fractions1,fractions[i])
        end
    end
    frac_variable = deepcopy(frac_variable1)
    fractions = deepcopy(fractions1)
    # select a fractional variable to branch (closest to 0.5)
    branch_value = [abs(fractions[i]-0.5) for i in 1:length(fractions)]
    idx_var = []
    aa = minimum(branch_value)
    for i in 1:length(branch_value)
        if branch_value[i] == aa
            push!(idx_var,i)
        end
    end
    idx_var_to_branch = rand(idx_var)
    var_to_branch = frac_variable[idx_var_to_branch]
    return ["flow_var", var_to_branch]
end

function integer_or_not(fraction_list)
    optimal_or_not = true
    for i in 1:length(fraction_list)
        #if floor(fraction_list[i]) != round.(fraction_list[i],digits=4) && ceil(fraction_list[i]) != round.(fraction_list[i],digits=4)
        if round.(fraction_list[i],digits=4)%1 != 0.0
            return false
        else
            optimal_or_not = true
        end
    end
    return optimal_or_not
end
