using Test
using MathOptInterface
using MathOptSetDistances
using FeasibilityOptInterface
using JuMP

const FOI = FeasibilityOptInterface
const MOD = MathOptSetDistances
const MOI = MathOptInterface
const MOIU = MOI.Utilities

@testset "Feasibility checker" begin

@testset "From primal start simple" begin
    mock = MOIU.MockOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()))
    # mock = MOIU.MockOptimizer(MOIU.Model{Float64}())

    MOIU.loadfromstring!(mock,
    """
        variables: x
        minobjective: 2.0x
        c1: x >= 1.0
        c2: x <= 2.0
    """)

    x = MOI.get(mock, MOI.VariableIndex, "x")

    for val in [0.5, 1.0, 1.5, 2.0, 2.5]
        MOI.set(mock, MOI.VariablePrimalStart(), x, val)
        @test MOI.get(mock, MOI.VariablePrimalStart(), x) == val

        largest, list = FOI.constraint_violation(mock, varval = FOI.variable_primal_start(mock))
        @test length(list) == 2
        @test largest == abs(val - clamp(val, 1.0, 2.0))

        ref = MOI.get(mock, MOI.ListOfConstraintIndices{
            MOI.SingleVariable, MOI.LessThan{Float64}}())[1]
        largest = FOI.constraint_violation(mock, ref, varval = FOI.variable_primal_start(mock))
        @test largest == abs(val - min(val, 2.0))

        largest, ref = FOI.constraint_violation(mock,
            MOI.SingleVariable, MOI.LessThan{Float64},
            varval = FOI.variable_primal_start(mock))
        @test largest == abs(val - min(val, 2.0))

        ref = MOI.get(mock, MOI.ListOfConstraintIndices{
            MOI.SingleVariable, MOI.GreaterThan{Float64}}())[1]
        largest = FOI.constraint_violation(mock, ref, varval = FOI.variable_primal_start(mock))
        @test largest == abs(val - max(val, 1.0))

        largest, ref = FOI.constraint_violation(mock,
            MOI.SingleVariable, MOI.GreaterThan{Float64},
            varval = FOI.variable_primal_start(mock))
        @test largest == abs(val - max(val, 1.0))

        largest, ref = FOI.constraint_violation(mock,
            MOI.SingleVariable, MOI.EqualTo{Float64},
            varval = FOI.variable_primal_start(mock))
        @test isnan(largest)
        @test ref === nothing

    end

    str = FOI.constraint_violation_report(FOI.FeasibilityChecker(mock, varval = FOI.variable_primal_start(mock)))

    # println(str)

    str_ans = """
    Primal Feasibility report
        tol = 0.0

    * Maximum overall violation = 0.5

    * Maximum violation per constraint type

     (SingleVariable, GreaterThan{Float64}) = 0.0
         Index: ConstraintIndex{SingleVariable,GreaterThan{Float64}}(12345679)
         Name: c1
     (SingleVariable, LessThan{Float64}) = 0.5
         Index: ConstraintIndex{SingleVariable,LessThan{Float64}}(12345679)
         Name: c2
    
    """

    @test str == str_ans
end

@testset "From primal start sum" begin
    mock = MOIU.MockOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()))
    # mock = MOIU.MockOptimizer(MOIU.Model{Float64}())

    MOIU.loadfromstring!(mock,
    """
        variables: x, y
        minobjective: 2.0x
        c1: x + 1*y >= 1.0
        c2: x + (-1)*y <= 2.0
    """)

    x = MOI.get(mock, MOI.VariableIndex, "x")
    y = MOI.get(mock, MOI.VariableIndex, "y")

    MOI.set(mock, MOI.VariablePrimalStart(), x, 2.0)
    @test MOI.get(mock, MOI.VariablePrimalStart(), x) == 2.0

    MOI.set(mock, MOI.VariablePrimalStart(), y, -2.0)
    @test MOI.get(mock, MOI.VariablePrimalStart(), y) == -2.0

    largest, ref = FOI.constraint_violation(mock,
        MOI.ScalarAffineFunction{Float64}, MOI.LessThan{Float64},
        varval = FOI.variable_primal_start(mock))
    @test largest == 2.0

    largest, ref = FOI.constraint_violation(mock,
        MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64},
        varval = FOI.variable_primal_start(mock))
    @test largest == 1.0
end

@testset "report" begin
    mock = MOIU.MockOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()))
    # mock = MOIU.MockOptimizer(MOIU.Model{Float64}())

    MOIU.loadfromstring!(mock,
    """
        variables: x, y
        minobjective: 2.0x
        c1: x + 1*y >= 1.0
        c2: x + (-1)*y <= 2.0
        c3: 0*x + 1*y >= 0.0
        c4: 1*x + 0*y >= 0.0
    """)

    # x = MOI.get(mock, MOI.VariableIndex, "x")
    # y = MOI.get(mock, MOI.VariableIndex, "y")

    # MOI.set(mock, MOI.VariablePrimalStart(), x, 2.0)
    # @test MOI.get(mock, MOI.VariablePrimalStart(), x) == 2.0

    MOIU.set_mock_optimize!(mock,
         (mock::MOIU.MockOptimizer) -> MOIU.mock_optimize!(mock, [0.0, 1.0],
            (MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}) => [0.0, 0.0, 2.0],
            (MOI.ScalarAffineFunction{Float64}, MOI.LessThan{Float64})    => [0.0],
            # (MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64})    => [0.0],
            # (MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64})    => [2.0],
         )
    )

    MOI.optimize!(mock)
    str = FOI.report(FOI.FeasibilityChecker(mock, check_dual = true))
    # println(str)

    str_ans = """
    Objective report

    * Problem sense: MIN_SENSE
    
    * Primal objective = 0.0
    
    * Dual objective = 0.0
    
    * Objective Gap = 0.0 [ |p-d|/(|p|+|d|+1e-10) ]
    
    Primal Feasibility report
        tol = 0.0
    
        Feasible within tol
    
    * Maximum overall violation = 0.0
    
    * Maximum violation per constraint type
    
     (ScalarAffineFunction{Float64}, GreaterThan{Float64}) = 0.0
         Index: ConstraintIndex{ScalarAffineFunction{Float64},GreaterThan{Float64}}(12345674)
         Name: c4
     (ScalarAffineFunction{Float64}, LessThan{Float64}) = 0.0
         Index: ConstraintIndex{ScalarAffineFunction{Float64},LessThan{Float64}}(12345676)
         Name: c2
    
    Dual Feasibility report
        tol = 0.0
    
        Feasible within tol
    
    * Maximum overall violation = 0.0
    
    * Maximum violation per constraint type
    
     (ScalarAffineFunction{Float64}, EqualTo{Float64}) = 0.0
         Index: ConstraintIndex{ScalarAffineFunction{Float64},EqualTo{Float64}}(2)
         Name: 
     (SingleVariable, GreaterThan{Float64}) = 0.0
         Index: ConstraintIndex{SingleVariable,GreaterThan{Float64}}(3)
         Name: 
     (SingleVariable, LessThan{Float64}) = 0.0
         Index: ConstraintIndex{SingleVariable,LessThan{Float64}}(4)
         Name: 
    
    Primal Complement report
    
    * Complement sum = 0.0
    
    * Largest complement violation = 0.0
    
    * Largest complement violation ctr = ConstraintIndex{ScalarAffineFunction{Float64},LessThan{Float64}}(12345676)
    
    Dual Complement report
    
    * Complement sum = 0.0
    
    * Largest complement violation = 0.0
    
    * Largest complement violation ctr = VariableIndex(12345676)

    """

    @test str == str_ans

end

@testset "JuMP bounds" begin
    model = Model()

    @variable(model, y == 0)
    @variable(model, 0 <= x <= 1)

    solution = Dict(y => 3.0, x => 2)

    varmap = var->solution[var]

    ck = FOI.FeasibilityChecker(model, varval = varmap)

    @test FOI.constraint_violation(ck, FixRef(y)) == 3.0
    @test FOI.constraint_violation(ck, UpperBoundRef(x)) == 1
    @test FOI.constraint_violation(ck, LowerBoundRef(x)) == 0

end
@testset "JuMP constraints" begin
    model = Model()

    @variable(model, x)
    @constraint(model, c1, x == 0)
    @constraint(model, c2, x == 1)
    @constraint(model, c3, x >= 2)
    @constraint(model, c4, x <= -2)
    @constraint(model, c5, x^2 <= -2)
    @constraint(model, c6, x^2 >= 2)

    solution = Dict(x => 0)

    varmap = var->solution[var]

    ck = FOI.FeasibilityChecker(model, varval = varmap)

    @test FOI.constraint_violation(ck, c1) == 0
    @test FOI.constraint_violation(ck, c2) == 1
    @test FOI.constraint_violation(ck, c3) == 2
    @test FOI.constraint_violation(ck, c4) == 2
    @test FOI.constraint_violation(ck, c5) == 2
    @test FOI.constraint_violation(ck, c6) == 2

end
@testset "JuMP report" begin
    model = Model()

    @variable(model, y == 0)
    @variable(model, 0 <= x <= 1)

    solution = Dict(y => 3.0, x => 2)

    varmap = var->solution[var]

    ck = FOI.FeasibilityChecker(model, varval = varmap)

    # @test_throws ErrorException FOI.report(ck)
    str = FOI.constraint_violation_report(ck)
    # println(str)

    str_ans = """
    Primal Feasibility report
        tol = 0.0

    * Maximum overall violation = 3.0

    * Maximum violation per constraint type

     (SingleVariable, EqualTo{Float64}) = 3.0
         Index: ConstraintIndex{SingleVariable,EqualTo{Float64}}(1)
         Name: 
     (SingleVariable, GreaterThan{Float64}) = 0.0
         Index: ConstraintIndex{SingleVariable,GreaterThan{Float64}}(2)
         Name: 
     (SingleVariable, LessThan{Float64}) = 1.0
         Index: ConstraintIndex{SingleVariable,LessThan{Float64}}(2)
         Name: 
    
    """

    @test str == str_ans

end

end