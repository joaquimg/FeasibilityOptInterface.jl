
# MOI -> JuMP
# JuMP.constraint_ref_with_index(model, index)
# JuMP.VariableRef(model, index)
# JuMP -> MOI
# JuMP.index(var_ref)
# JuMP.index(con_ref)

# function _fix_ref(checker::FeasibilityChecker, ref::VI)
#     if checker.jump !== nothing
#         return JuMP.VariableRef(checker.jump, ref)
#     end
#     return ref
# end
# function _fix_ref(checker::FeasibilityChecker, ref::CI)
#     if checker.jump !== nothing
#         return JuMP.constraint_ref_with_index(checker.jump, ref)
#     end
#     return ref
# end

const JuMPRef = Union{JuMP.VariableRef, JuMP.ConstraintRef}

_moi_ref(ref::CI) = ref
_moi_ref(ref::VI) = ref
_moi_ref(ref::JuMPRef) = JuMP.index(ref)


#primal
function constraint_violation(checker::FeasibilityChecker,
primal_con_or_var::JuMPRef)
    constraint_violation(checker, JuMP.index(primal_con_or_var))
end

# dual
function dual_constraint_violation(checker::FeasibilityChecker,
    primal_con_or_var::JuMPRef)
    dual_constraint_violation(checker, JuMP.index(primal_con_or_var))
end

# complement
function complement_violation(checker::FeasibilityChecker,
    primal_con::JuMP.ConstraintRef)
    return complement_violation(checker, JuMP.index(primal_con))
end
function dual_complement_violation(checker::FeasibilityChecker,
    primal_var::JuMP.VariableRef)
    return dual_complement_violation(checker, JuMP.index(primal_var))
end