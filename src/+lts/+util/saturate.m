function value = saturate(value)
% SATURATE Constrain a value to the unit interval [0, 1].
value = lts.util.clamp(value, 0, 1);
end
