abstract type AbstractNode end
abstract type AbstractBranch end

push!(branches::Array{AbstractBranch,1}, b::AbstractBranch) = Base.pushfirst!(branches, b)
push!(branches::Array{AbstractBranch,1}, b::Nothing) = branches

macro abstract_node_fields()
    return esc(quote
        id::Int
        parent::Union{Nothing,AbstractNode}
        depth::Int
        #branch::Union{Nothing,T}
        branch::Vector{Any}
        bound::Float64 #如果bound=Inf,这表示is_rmp_optimal = false!
        solution::Vector{Any}
        fraction::Vector{Any}
        cols::Vector{Any}
        cost_cols::Vector{Any}
    end)
end

# empty branching function
branch(node::AbstractNode)::Vector{AbstractNode} = []

# empty bounding function
function bound!(node::AbstractNode) end

# empty heuristics function
function heuristics!(node::AbstractNode) end

# apply changes (branched information) from ancestor to node
function apply_changes!(node::AbstractNode, ancestor::AbstractNode) end

apply_changes!(node::AbstractNode) = apply_changes!(node, node)

#############
# BnP solve #
#############
mutable struct JuMPNode{T<:AbstractBranch} <: AbstractNode
    @abstract_node_fields

    function JuMPNode{T}(
        parent = nothing,
        depth = 0,
        bound = -1000, #如果bound=Inf,这表示is_rmp_optimal = false!
        solution = [],
        fraction = [],
        cols = [],
        cost_cols = [],
        id = -1,
        branch = []
        #branch = nothing # branch decision: [["total veh"],["total_recharge"],etc..]
        ) where T<:AbstractBranch
        return new{T}(id, parent, depth, branch, bound, solution,fraction, cols, cost_cols)
    end
end

JuMPNode{T}(parent::JuMPNode{T}) where T<:AbstractBranch = JuMPNode{T}(parent, parent.depth + 1, parent.bound, [], [], parent.cols, parent.cost_cols) #记录parent到子节点
JuMPNode(parent::JuMPNode{T}) where T<:AbstractBranch = JuMPNode{T}(parent)
