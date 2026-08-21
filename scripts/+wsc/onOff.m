function value = onOff(flag, trueValue, falseValue)
%ONOFF Pick one of two values by a logical flag.
if flag
    value = trueValue;
else
    value = falseValue;
end
end
