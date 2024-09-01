#########################
# Neighborhood prunning #
#########################
function create_permutation(i,j) #(i,j)
    k = 1
    a = [[P_d[k], i, i+numb_user, j, j+numb_user, D_d[k]],
        [P_d[k], i, j, i+numb_user, j+numb_user, D_d[k]],
        [P_d[k], i, j, j+numb_user, i+numb_user, D_d[k]],
        [P_d[k], j, i, j+numb_user, i+numb_user, D_d[k]],
        [P_d[k], j, i, i+numb_user, j+numb_user, D_d[k]],
        [P_d[k], j, j+numb_user, i, i+numb_user, D_d[k]]]
    return a
end
function preliminary_check(i,j)
    paths = create_permutation(i,j)
    values = []
    for ii in 1:length(paths)
        push!(values, is_feasible_tour(paths[ii],k))
    end
    return values
end

function create_conflict_graph()
    k = 1
    graph = ones(numb_nodes,numb_nodes)
    for i in union(P_u,D_u)
        for j in union(P_u,D_u)
            if e[i] + s[i] + t[i,j] > l[j]
                graph[i,j] = 0
            end
        end
    end

    for i in P_u
        for j in union(P_u,D_u)
            if t[i,j] + s[j] + t[j,n+i] > m[i]
                graph[i,j] = 0
                graph[j,n+i] = 0
            end
        end
    end

    for i in P_u # pickup node
        # arc (dropoff, pickup) is infeasible
        graph[numb_user+i,i] = 0
        for j in i+1:numb_user
            res = preliminary_check(i,j)
            if res[1] == false #checked
                graph[numb_user+i,j] = 0
            end
            if res[6] == false
                graph[numb_user+j,i] = 0
            end
            if res[2] == false
                graph[j,numb_user+i] = 0
            end
            if res[4] == false #checked
                graph[i,numb_user+j] = 0
            end
            if res[2] == false && res[3] == false#checked
                graph[i,j] = 0
            end
            if res[4] == false && res[5] == false#checked
                graph[j,i] = 0
            end
            if res[2] == false && res[5] == false#checked
                graph[numb_user+i,numb_user+j] = 0
            end
            if res[4] == false && res[3] == false
                graph[numb_user+j,numb_user+i] = 0
            end
        end
    end
    return graph
end
