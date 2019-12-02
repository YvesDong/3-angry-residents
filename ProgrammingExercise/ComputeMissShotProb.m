function p_tri_miss = ComputeMissShotProb(shooter,m,n)
global R GAMMA
d = [];
for i = 1:size(shooter,1)
    d = [d, abs(shooter(i,1) - m)+abs(shooter(i,2) - n)];
end

p_miss =zeros(1,3);

for i = 1:size(d,2)
    if d(i) < R+1
        p_miss(i) = 1-GAMMA/double((d(i)+1));
    else
        p_miss(i) = 1;
    end
end

p_tri_miss = prod(p_miss,2);

end