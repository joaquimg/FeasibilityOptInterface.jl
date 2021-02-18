module FeasibilityOptInterface

import Dualization
import JuMP
import LinearAlgebra
import MathOptInterface
import MathOptSetDistances

const MOD = MathOptSetDistances
const MOI = MathOptInterface
const MOIU = MOI.Utilities

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

const CR{F,S} = JuMP.ConstraintRef{JuMP.Model, CI{F,S}}

include("checker.jl")
include("primal.jl")
include("dual.jl")
include("complementarity.jl")
include("jump.jl")

end # module
