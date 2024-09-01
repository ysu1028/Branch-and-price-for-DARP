############
# BnP tree #
############
mutable struct AbstractTree
    node_counter::Int
    nodes::Vector{AbstractNode}
    processed::Vector{AbstractNode}
    best_bound::Float64     # e.g., lower bound for minimization
    best_incumbent::Float64 # e.g., upper bound for minimization

    function AbstractTree(node_counter::Int = 0)
        return new(node_counter, [], [], -Inf, Inf)
    end
end

# basic termination function
termination(tree::AbstractTree) = isempty(tree)
isempty(tree::AbstractTree) = Base.isempty(tree.nodes)

# add node to tree
function push!(tree::AbstractTree, node::AbstractNode)
    tree.node_counter += 1
    node.id = tree.node_counter
    Base.push!(tree.nodes, node)
    sort!(tree)
end

# add a set of nodes to tree
function push!(tree::AbstractTree, nodes::Vector{T}) where T<:AbstractNode
    for node in nodes
        push!(tree, node)
    end
end

# mark node as processed
processed!(tree::AbstractTree, node::AbstractNode) = Base.push!(tree.processed, node)

function update_best_bound!(tree::AbstractTree)
    if !isempty(tree)
        tree.best_bound = Base.minimum([node.bound for node in tree.nodes])
    end
end

function branch!(tree::AbstractTree, node::AbstractNode)
    children = branch(node)
    push!(tree, children)
end

# return the next search node: can be improved
function next_node(tree::AbstractTree)
    sort!(tree)
    node = Base.pop!(tree.nodes)
    apply_changes!(node)
    return node
end

# best bound
sort!(tree::AbstractTree) = Base.sort!(tree.nodes, by=x->x.bound, rev=true)

#############
# BnP solve #
#############
function initialize_tree(root::JuMPNode{T})::AbstractTree where T<:AbstractBranch
    tree = AbstractTree()
    push!(tree, root)
    return tree
end
initialize_tree(
    cols::Vector{Any},
    cost_cols::Vector{Any},
    frac_sol::Vector{Any},
    frac::Vector{Any},
    bound::Float64,
    T = AbstractBranch,
)::AbstractTree = initialize_tree(JuMPNode{T}(nothing,0,bound, frac_sol, frac, cols, cost_cols))

# Create child node with abstract branch object
function create_child_node(current_node::JuMPNode{T}, branch::Vector{Any}) where T<:AbstractBranch
    node = JuMPNode(current_node)
    node.branch = branch
    return node
end

function processed!(tree::AbstractTree, node::JuMPNode)
    Base.push!(tree.processed, node)
end
####################
# Branch and Price #
####################
function termination(tree::AbstractTree)
    @info "Tree nodes: processed $(length(tree.processed)), left $(length(tree.nodes)), total $(tree.node_counter), best bound $(tree.best_bound), best incumbent $(tree.best_incumbent)"
    if isempty(tree)
        @info "Completed the tree search"
        return true
    end
    # if length(tree.processed) >= maximum_numb_nodes
    #     @info "Reached node limit"
    #     return true
    # end
    return false
end
