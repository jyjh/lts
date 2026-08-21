function tf = validateUnitScalarOrEmpty(value)
%VALIDATEUNITSCALAROREMPTY Scalar in [0, 1], or empty.
if isempty(value)
    tf = true;
    return;
end
tf = wsc.validateUnitScalar(value);
end
