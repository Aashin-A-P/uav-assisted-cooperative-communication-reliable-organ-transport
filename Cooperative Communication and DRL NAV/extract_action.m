function a = extract_action(act)

if iscell(act)
    act = act{1};
end

if isa(act,'dlarray')
    act = extractdata(act);
end

a = double(act);

if numel(a) > 1
    a = a(1);
end

end