
function objective_report(checker::FeasibilityChecker)

    objval = objective(checker)

    sense = MOI.get(checker.primal, MOI.ObjectiveSense())::MOI.OptimizationSense

    str =
    """
    Objective report

    * Problem sense: $(sense)

    * Primal objective = $(objval)

    """
    
    if checker.check_dual
        _has_dual_model(checker)

        d_objval = dual_objective(checker)
        gap = _objective_gap(checker, objval, d_objval)

        str *= """
        * Dual objective = $(d_objval)

        * Objective Gap = $(gap) [ |p-d|/(|p|+|d|+1e-10) ]

        """
    end

    return _remove_moi(str, checker)
end

function objective_gap(checker::FeasibilityChecker)
    objval = objective(checker)
    d_objval = dual_objective(checker)
    return _objective_gap(checker, objval, d_objval)
end
function _objective_gap(::FeasibilityChecker, objval, d_objval)
    return abs(objval - d_objval) / (abs(objval) + abs(d_objval) + 1e-10)
end

function objective(checker::FeasibilityChecker)
    tp_obj = MOI.get(checker.primal, MOI.ObjectiveFunctionType())
    @assert tp_obj !== nothing
    obj = MOI.get(checker.primal, MOI.ObjectiveFunction{tp_obj}())
    return MOIU.eval_variables(checker.varval, obj)
end

function dual_objective(checker::FeasibilityChecker)
    tp_obj = MOI.get(checker.dual.dual_model, MOI.ObjectiveFunctionType())
    @assert tp_obj !== nothing
    obj = MOI.get(checker.dual.dual_model, MOI.ObjectiveFunction{tp_obj}())
    dual_varval = x -> _dual_variable_primal(checker, x)
    return MOIU.eval_variables(dual_varval, obj)
end

function complement_violation_report(checker::FeasibilityChecker)

    total, largest, largest_ref = complement_violation(checker)

    str =
    """
    Primal Complement report

    * Complement sum = $(total)

    * Largest complement violation = $(largest)

    * Largest complement violation ctr = $(largest_ref)

    """

    return _remove_moi(str, checker)
end

function complement_violation(checker::FeasibilityChecker)
    total = 0.0
    largest = 0.0
    largest_ref = nothing
    for (F, S) in MOI.get(checker.primal, MOI.ListOfConstraints())
        list = MOI.get(checker.primal, MOI.ListOfConstraintIndices{F, S}())
        for con in list
            val = complement_violation(checker, con)
            total += val
            if val >= largest
                largest = val
                largest_ref = con #_fix_ref(checker, con)
            end
        end
    end
    return total, largest, largest_ref
end

function complement_violation(checker::FeasibilityChecker,
    primal_con::CI{F, S}) where {F, S}
    func = _function_with_constant(checker.primal, primal_con)
    primal_con_val = MOIU.eval_variables(checker.varval, func)
    dual_var_val = checker.condual(primal_con)
    prod = LinearAlgebra.dot(primal_con_val, dual_var_val)
    return abs(prod)
end

function dual_complement_violation_report(checker::FeasibilityChecker)
    _has_dual_model(checker)

    total, largest, largest_ref = dual_complement_violation(checker)

    str =
    """
    Dual Complement report

    * Complement sum = $(total)

    * Largest complement violation = $(largest)

    * Largest complement violation ctr = $(largest_ref)

    """

    return _remove_moi(str, checker)
end

function dual_complement_violation(checker::FeasibilityChecker)
    _has_dual_model(checker)
    total = 0.0
    largest = 0.0
    largest_ref = nothing
    for vi in MOI.get(checker.primal, MOI.ListOfVariableIndices())
        val = dual_complement_violation(checker, vi)
        total += val
        if val >= largest
            largest = val
            largest_ref = vi #_fix_ref(checker, vi)
        end
    end
    return total, largest, largest_ref
end

function dual_complement_violation(checker::FeasibilityChecker,
    primal_var::MOI.VariableIndex)
    _has_dual_model(checker)

    primal_var_val = checker.varval(primal_var)
    dual_con = checker.dual.primal_dual_map.primal_var_dual_con[primal_var]

    func = _function_with_constant(checker.dual.dual_model, dual_con)

    dual_varval = x -> _dual_variable_primal(checker, x)

    dual_con_val = MOIU.eval_variables(dual_varval, func)

    prod = LinearAlgebra.dot(primal_var_val, dual_con_val)

    return abs(prod)
end

function _function_with_constant(model, ci::CI{F, S}
) where {F, S<:MOI.AbstractVectorSet}
    return MOI.get(model, MOI.ConstraintFunction(), ci)::F
end
function _function_with_constant(model, ci::CI{F,S}
) where {F, S<:MOI.AbstractScalarSet}
    T = Float64
    func = MOI.copy(MOI.get(model, MOI.ConstraintFunction(), ci))::F
    set = MOI.copy(MOI.get(model, MOI.ConstraintSet(), ci))::S
    constant = Dualization.set_dot(1, set, T) *
        Dualization.get_scalar_term(model, 1, ci)
    if F == MOI.SingleVariable
        func = MOIU.operate(+, T, func, constant)
    else
        func.constant = constant
    end
    return func
end