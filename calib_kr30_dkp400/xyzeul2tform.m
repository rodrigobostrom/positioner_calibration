function T = xyzeul2tform(v)


[n,m] = size(v);

if (m ~= 6)
    error('Column size should be equal to 6.');
end


T = [];

for i = 1:n
    vi = v(i,:);
    
    pos = vi(1:3);
    ori = eul2rotm(vi(4:6),'zyx');
    
    T(:,:,i) = [ori pos(:); 0 0 0 1];
end
