function dual_constraint_violation_report(checker::FeasibilityChecker)
    constraint_violation_report(checker, false)
end

function dual_constraint_violation(checker::FeasibilityChecker)
    _has_dual_model(checker)
    return constraint_violation(checker.dual.dual_model,
        varval = x -> _dual_variable_primal(checker, x),
        distance_map = checker.distance_map)
end
function dual_constraint_violation(checker::FeasibilityChecker, ::Type{F}, ::Type{S}
) where {F, S}
    _has_dual_model(checker)
    return constraint_violation(checker.dual.dual_model, F, S,
        varval = x -> _dual_variable_primal(checker, x),
        distance = _get_distance(checker.distance_map, F, S))
end
function dual_constraint_violation(checker::FeasibilityChecker, primal_con_or_var)
    _has_dual_model(checker)
    dual_con = _get_dual_con(checker::FeasibilityChecker, primal_con_or_var)
    distance = _get_distance(checker.distance_map, dual_con)

    # dual constraint data
    func = MOI.get(checker.dual.dual_model, MOI.ConstraintFunction(), dual_con)
    set  = MOI.get(checker.dual.dual_model, MOI.ConstraintSet(), dual_con)

    # dual_variable -> primal_constraint
    dual_varval = x -> _dual_variable_primal(checker, x)

    val  = MOIU.eval_variables(dual_varval, func)
    dist = MOD.distance_to_set(distance, val, set)

    return dist
end

function _get_dual_con(checker::FeasibilityChecker, primal_con::CI)
    return checker.dual.primal_dual_map.primal_con_dual_con[primal_con]
end
function _get_dual_con(checker::FeasibilityChecker, primal_var::VI)
    return checker.dual.primal_dual_map.primal_var_dual_con[primal_var]
end

function _dual_variable_primal(checker, dual_vi)
    # which is the corresponding primal constraint dual
    (con, ind) = checker.dual_vi_primal_con[dual_vi]
    if ind > 0
        return checker.condual(con)[ind]
    else
        return checker.condual(con)
    end
end

function _build_dual_var_map(dual)
    dict = Dict{VI, Tuple{CI, Int}}()
    # sizehint!(dict, ???)
    for (ci, vec_var) in dual.primal_dual_map.primal_con_dual_var
        if _is_vec(ci)
            for (i, vi) in enumerate(vec_var)
                dict[vi] = (ci, i)
            end
        else
            dict[vec_var[]] = (ci, 0)
        end
    end
    return dict
end
function _build_dual_var_map(::Nothing)
    return nothing
end

function _is_vec(::CI{F, S}) where {F, S <: MOI.AbstractVectorSet}
    return true
end
function _is_vec(::CI{F, S}) where {F, S <: MOI.AbstractScalarSet}
    return false
end
