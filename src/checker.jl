variable_primal(model) = x -> MOI.get(model, MOI.VariablePrimal(), x)
variable_primal_start(model) = x -> MOI.get(model, MOI.VariablePrimalStart(), x)

constraint_dual(model) = x -> MOI.get(model, MOI.ConstraintDual(), x)
constraint_dual_start(model) = x -> MOI.get(model, MOI.ConstraintDual(), x)

struct EmptyDict{K,V} <: AbstractDict{K,V}
end
EmptyDict() = EmptyDict{Any,Any}()
haskey(::EmptyDict, key) = false


"""
    options

* `varval` can be used to define a map `vi -> value` where `vi` is a
`MOI.VariableIndex` in the model. This map should be defined for all variables
tha appear in the constraint.

* `distance_map` is an abstract dictionary mapping function-set pairs `(F, S)`
to `distance`s that are instances of `AbstractDistance`. The function
`distance_to_set(distance, v, set)` must be implemented for this type for the
set of the constraint `con`. Is some function-set pair `(F, S)` is not found in
the keys of the dictionay then the MOD.DefaultDistance() is used.

* `tol` is used to ignore violations larger than its value.

* `names`: is false, constraint names are not displayed in the violation report.

* `index`: is false, constraint indexes are not displayed in the violation report.

"""
mutable struct FeasibilityChecker
    primal::MOI.ModelLike
    check_dual::Bool
    dual::Union{Dualization.DualProblem, Nothing}
    dual_vi_primal_con::Union{Dict, Nothing}
    distance_map::AbstractDict
    tol::Number
    varval::Function
    condual::Function
    # options::FeasibilityCheckerOptions

    primal_tol::Number
    dual_tol::Number
    complement_tol::Number
    objective_tol::Number

    remove_moi_str::Bool

    print_below_tol::Bool
    print_names::Bool
    print_index::Bool
    print_distance::Bool

    # add tolerance per constraint type ?
    # complement bound is in sum. should it be individual?
end
function FeasibilityChecker(model::MOI.ModelLike;
    check_dual::Bool = true,

    distance_map::AbstractDict = EmptyDict(),
    varval::Function = variable_primal(model),
    condual::Function = constraint_dual(model),

    tol::Real = 0.0,
    primal_tol::Real = -1.0,
    dual_tol::Real = -1.0,
    complement_tol::Real = -1.0,
    objective_tol::Real = -1.0,

    remove_moi_str::Bool = true,
    print_below_tol::Bool = true,
    print_names::Bool = true,
    print_index::Bool = true,
    print_distance::Bool = true,
)
    return FeasibilityChecker(
        model,
        check_dual,
        nothing,
        nothing,
        distance_map,
        tol,
        varval,
        condual,
        # options,
        primal_tol,
        dual_tol,
        complement_tol,
        objective_tol,
        remove_moi_str,
        print_below_tol,
        print_names,
        print_index,
        print_distance,    
    )
end

# lazy loading of dual model
function _load_dual(checker::FeasibilityChecker)
    if checker.check_dual && checker.dual === nothing
        checker.dual = Dualization.dualize(checker.primal)
        checker.dual_vi_primal_con = _build_dual_var_map(checker.dual)
    end
end

function _has_dual_model(checker::FeasibilityChecker)
    _load_dual(checker)
    if checker.dual === nothing
        error(
            "FeasibilityChecker dual model was disabled with `check_dual`. Re-enable this option if you want to use this function."
        )
    end
end

function primal_dual_constraint_violation_report(checker::FeasibilityChecker)
    _has_dual_model(checker)
    return constraint_violation_report(checker) *
        dual_constraint_violation_report(checker)
end

function report(checker::FeasibilityChecker)
    _has_dual_model(checker)
    str = ""
    str *= objective_report(checker)
    str *= primal_dual_constraint_violation_report(checker)
    str *= complement_violation_report(checker)
    str *= dual_complement_violation_report(checker)
    return _remove_moi(str, checker)
end

function is_primal_feasible(checker::FeasibilityChecker)
    largest, _ = constraint_violation(checker::FeasibilityChecker)
    return largest < _primal_tol(checker)
end
function is_dual_feasible(checker::FeasibilityChecker)
    _has_dual_model(checker)
    largest, _ = dual_constraint_violation(checker::FeasibilityChecker)
    return largest < _dual_tol(checker)
end

function is_complement(checker::FeasibilityChecker)
    _has_dual_model(checker)
    total_p, _, _ = complement_violation(checker)
    total_d, _, _ = dual_complement_violation(checker)
    return total_p + total_d < _complement_tol(checker)
end

function is_zero_gap(checker::FeasibilityChecker)
    _has_dual_model(checker)
    return objective_gap(checker) < _objective_tol(checker)
end

function is_optimal(checker::FeasibilityChecker)
    _has_dual_model(checker)
    return is_primal_feasible(checker) &&
           is_dual_feasible(checker) &&
           is_complement(checker) &&
           is_zero_gap(checker)
end


function _primal_tol(checker::FeasibilityChecker)
    if checker.primal_tol < 0
        return checker.tol
    else
        return checker.primal_tol
    end
end
function _dual_tol(checker::FeasibilityChecker)
    if checker.dual_tol < 0
        return checker.tol
    else
        return checker.dual_tol
    end
end
function _complement_tol(checker::FeasibilityChecker)
    if checker.complement_tol < 0
        return checker.tol
    else
        return checker.complement_tol
    end
end
function _objective_tol(checker::FeasibilityChecker)
    if checker.objective_tol < 0
        return checker.tol
    else
        return checker.objective_tol
    end
end

function _remove_moi(str, remove::Bool)
    if remove
        return replace(str, "MathOptInterface." => "")
    else
        return str
    end
end
function _remove_moi(str, checker::FeasibilityChecker)
    _remove_moi(str, checker.remove_moi_str)
end