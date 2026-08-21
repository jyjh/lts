function tf = validateScalarLogical(value)
%VALIDATESCALARLOGICAL Scalar logical or numeric flag.
validateattributes(value, {'logical', 'numeric'}, ...
    {'scalar'}, mfilename);
tf = true;
end
