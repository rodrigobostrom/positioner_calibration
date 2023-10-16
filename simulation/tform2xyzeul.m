function V = tform2xyzeul(T)

[~,~,n] = size(T);

V = zeros(n,6);

for i = 1:n
    
    Ti = T(:,:,i);
    
    pos = Ti(1:3,4);
    ori = rotm2eul(Ti(1:3,1:3),'zyx');
    
    V(i,:) = [pos ori];
end
