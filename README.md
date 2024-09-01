# Branch-and-price-for-DARP
The branch-and-price framework for Dial-a-ride problem.

# Instance
We take the smallest instance of Cordeau (2006) as an example
The instance (a2-16-0.7) consists of two vehicles and 16 user requests. It has the following format:

Instance names: <a/u><numvehicles>-<numcustomers>-<minimum end battery ratio levels>

#vehicles #users #origindepots #destinationdepots #stations #replications #time horizon
node_id lat long service time load arr dep
...
common origin depot id
common destination depot id
(artificial) origin depots id
(artificial) destination depots id
charging stations id
users maximum ride time
vehicles capacity
vehicles initial battery inventory
vehicles battery capacities
minimum end battery ratio levels
recharging rates at charging stations
vehicles discharging rate
weight factors
original travel times

# MILP (/utils/milp.jl) to solve the pricing subproblem
We take the example of a milp solver + callback function to generate multiple columns with negative reduced cost
