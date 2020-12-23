module FeasibilityOptInterface

using LinearAlgebra
using MathOptInterface
using MathOptSetDistances

using Dualization

const MOD = MathOptSetDistances
const MOI = MathOptInterface
const MOIU = MOI.Utilities

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

include("checker.jl")
include("primal.jl")
include("dual.jl")
include("complementarity.jl")

end # module
