function constraint_violation(checker::FeasibilityChecker)
    return constraint_violation(checker.primal,
        varval = checker.varval,
        distance_map = checker.distance_map)
end
function constraint_violation(checker::FeasibilityChecker, ::Type{F}, ::Type{S}
) where {F, S}
    return constraint_violation(checker.primal, F, S,
        varval = checker.varval,
        distance = _get_distance(checker.distance_map, F, S))
end
function constraint_violation(checker::FeasibilityChecker,
    con::MOI.ConstraintIndex{F, S}
) where {F, S}
    return constraint_violation(checker.primal, con;
        varval = checker.varval,
        distance = _get_distance(checker.distance_map, F, S))
end

"""
    constraint_violation_report(checker::FeasibilityChecker)

Given a Model `model` return a string with a detailed report of the constraint
violations in the problem.
"""
function constraint_violation_report(checker::FeasibilityChecker, primal = true)
    if primal
        tol = _primal_tol(checker)
        largest, vec = constraint_violation(checker)
        model = checker.primal
    else
        tol = _dual_tol(checker)
        largest, vec = dual_constraint_violation(checker)
        model = checker.dual.dual_model
    end

    str =
        """
        $(primal ? "Primal" : "Dual") Feasibility report
            tol = $tol
        
        """
    if largest <= tol
        str *=
        """
            Feasible within tol

        """
    end
    if largest > tol || checker.print_below_tol
        str *=
        """
        * Maximum overall violation = $(largest)

        * Maximum violation per constraint type

        """
    end

    # sort!(vec)
    has_line = false
    for (val, ref) in vec
        if val >= tol || checker.print_below_tol
            str *= _violation_string(model, ref, val, checker)
            has_line = true
        end
    end

    if has_line
        str *= "\n"
    end

    return _remove_moi(str, checker)
end

"""
    _violation_string(model, ref::MOI.ConstraintIndex{F, S},
                      val, distance_map, options)

Internal method to build lines of the `constraint_violation_report`.
"""
function _violation_string(model, ref::MOI.ConstraintIndex{F, S},
    val, checker
) where {F, S}
    str = " ($F, $S) = $(val)"
    if haskey(checker.distance_map, (F, S)) && checker.print_distance
        str *= " [$(distance_map[F, S])]"
    end
    str *= "\n"
    if checker.print_index
        str *= "     Index: $(ref)\n"
    end
    if checker.print_names && MOI.supports(model, MOI.ConstraintName(),
                                     MOI.ConstraintIndex{F, S})
        name = try
            MOI.get(model, MOI.ConstraintName(), ref)
        catch
            nothing
        end
        if name === nothing 
            str *= "     Name: $(name)\n"
        end
    end
    return str
end

"""
    constraint_violation(model::MOI.ModelLike)

Given a Model `model` return the largest violation value of all constraints and 
a vector with tuples, with the largest violation by constraint type and the
constraint that attains that violation value.

    constraint_violation(model::MOI.ModelLike;
                         varval::Function = variable_primal(model),
                         distance_map::MOD.AbstractDistance = MOD.DefaultDistance())

Expanded method to prodive more flexibility to the user.

* `varval` can be used to define a map `vi -> value` where `vi` is a
`MOI.VariableIndex` in the model. This map should be defined for all variables
tha appear in the constraint.

* `distance_map` is an abstract dictionary mapping function-set pairs `(F, S)`
to `distance`s that are instances of `AbstractDistance`. The function
`distance_to_set(distance, v, set)` must be implemented for this type for the
set of the constraint `con`. Is some function-set pair `(F, S)` is not found in
the keys of the dictionay then the MOD.DefaultDistance() is used.
"""
function constraint_violation(model::MOI.ModelLike;
    varval::Function = variable_primal(model),
    distance_map::AbstractDict = EmptyDict()
)
    vec = Any[]
    largest = 0.0
    for (F, S) in MOI.get(model, MOI.ListOfConstraints())
        distance = _get_distance(distance_map, F, S)
        val, ref = constraint_violation(model, F, S, varval = varval, distance = distance)
        push!(vec, (val, ref))
        if val >= largest
            largest = val
        end
    end
    return largest, vec
end

function _get_distance(::EmptyDict, ::Type{F}, ::Type{S}) where {F, S}
    return MOD.DefaultDistance()
end
function _get_distance(::EmptyDict, ::CI{F, S}) where {F, S}
    return MOD.DefaultDistance()
end
function _get_distance(distance_map, ::Type{F}, ::Type{S}) where {F, S}
    if haskey(distance_map, (F, S))
        return distance_map[F, S]
    else
        return MOD.DefaultDistance()
    end
end
function _get_distance(distance_map, ::CI{F, S}) where {F, S}
    if haskey(distance_map, (F, S))
        return distance_map[F, S]
    else
        return MOD.DefaultDistance()
    end
end


"""
    constraint_violation(model::MOI.ModelLike, F, S)

Given a Model `model` and one funtions `F` and set `S` pair, return the largest
violation value and the reference to the constraint with largest violation of
the constraint type define by the pair with respect the currently
available solution and the default distance for the constraint set.

    constraint_violation(model::MOI.ModelLike, F, S;
                         varval::Function = variable_primal(model),
                         distance::MOD.AbstractDistance = MOD.DefaultDistance())

Expanded method to prodive more flexibility to the user.

* `varval` can be used to define a map `vi -> value` where `vi` is a
`MOI.VariableIndex` in the model. This map should be defined for all variables
tha appear in the constraint.

* `distance` is an instance of `AbstractDistance`. The function
`distance_to_set(distance, v, set)` must be implemented for this type for the
set of the constraint `con`.
"""
function constraint_violation(model::MOI.ModelLike,
    ::Type{F}, ::Type{S};
    varval::Function = variable_primal(model),
    distance::MOD.AbstractDistance = MOD.DefaultDistance()
) where {F, S}
    largest = 0.0
    largest_ref = MOI.ConstraintIndex{F, S}(-1)
    list = MOI.get(model, MOI.ListOfConstraintIndices{F, S}())
    if isempty(list)
        return NaN, nothing
    end
    for con in list
        val = constraint_violation(model, con, varval = varval, distance = distance)
        if val >= largest
            largest = val
            largest_ref = con
        end
    end
    return largest, largest_ref
end

"""
    constraint_violation(model::MOI.ModelLike, con::MOI.ConstraintIndex)

Given a Model `model` and one of it's constraints `con`, return the violation of
the constraint with respect the currently available solution and the default
distance for the constraint set.

    constraint_violation(model::MOI.ModelLike, con::MOI.ConstraintIndex;
                         varval::Function = variable_primal(model),
                         distance::MOD.AbstractDistance = MOD.DefaultDistance())

Expanded method to prodive more flexibility to the user.

* `varval` can be used to define a map `vi -> value` where `vi` is a
`MOI.VariableIndex` in the model. This map should be defined for all variables
tha appear in the constraint.

* `distance` is an instance of `AbstractDistance`. The function
`distance_to_set(distance, v, set)` must be implemented for this type for the
set of the constraint `con`.
"""
function constraint_violation(model::MOI.ModelLike, con::MOI.ConstraintIndex;
    varval::Function = variable_primal(model),
    distance::MOD.AbstractDistance = MOD.DefaultDistance()
)
    func = MOI.get(model, MOI.ConstraintFunction(), con)
    set  = MOI.get(model, MOI.ConstraintSet(), con)
    val  = MOIU.eval_variables(varval, func)
    dist = distance_to_set(distance, val, set)
    return dist
end