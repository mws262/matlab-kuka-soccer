function splinepp = qp_collocation(breaks, knots, extraPointsPerSegment, velocityWeight)
% Insert collocation points (not fixed) between each defined knot point.
allTimes = breaks(1:end-1);
for i = 1:extraPointsPerSegment
    allTimes = [allTimes; diff(breaks) * i /(extraPointsPerSegment + 1) + breaks(1:end-1)];
end
allTimes = [allTimes(:); breaks(end)];
definedKnotIdx = 1:(extraPointsPerSegment + 1):size(allTimes,1); % Indices of collocation points which represent the fixed knots.

% Pick the number of collocation points.
numColPts = length(allTimes);
% Number of dimensions (e.g. x and y).
numDims = 1;

% Hermite-Simpson collocation -- have decision variables for half-way
% velocities and accelerations.
numDecisionVars = numDims * (5 * numColPts - 2);

% Fix the times for the collocation points.
dt = diff(allTimes);

% Determine where in the decision variable vector the various terms appear.
idx = 1;
xStartIdx = idx; idx = idx + numColPts;
vStartIdx = idx; idx = idx + numColPts;
aStartIdx = idx; idx = idx + numColPts;

vHalfStartIdx = idx; idx = idx + numColPts - 1;
aHalfStartIdx = idx; idx = idx + numColPts - 1;

% Allocate for the various collocation constraints (the A in Ax = b).
xCol = zeros(numColPts - 1, numDecisionVars);
vCol = zeros(numColPts - 1, numDecisionVars);
vHalfCol = zeros(numColPts - 1, numDecisionVars);

% Allocate for quadratic cost x'Hx
costH = zeros(numDecisionVars);

for i = 0:numColPts - 2
    % Position collocation constraints.
    xCol(i + 1, xStartIdx + i) = 1;
    xCol(i + 1, xStartIdx + i + 1) = -1;
    xCol(i + 1, vStartIdx + i) = dt(i + 1)/6;
    xCol(i + 1, vStartIdx + i + 1) = dt(i + 1)/6;
    xCol(i + 1, vHalfStartIdx + i) = 2*dt(i + 1)/3;

    % Velocity collocation constraints.
    vCol(i + 1, vStartIdx + i) = 1;
    vCol(i + 1, vStartIdx + i + 1) = -1;
    vCol(i + 1, aStartIdx + i) = dt(i + 1)/6;
    vCol(i + 1, aStartIdx + i + 1) = dt(i + 1)/6;
    vCol(i + 1, aHalfStartIdx + i) = 2*dt(i + 1)/3;
    
    % Half-way velocity collocation constraints.
    vHalfCol(i + 1, vStartIdx + i) = 1/2;
    vHalfCol(i + 1, vStartIdx + i + 1) = 1/2;
    vHalfCol(i + 1, aStartIdx + i) = dt(i + 1)/8;
    vHalfCol(i + 1, aStartIdx + i + 1) = -dt(i + 1)/8;
    vHalfCol(i + 1, vHalfStartIdx + i) = -1;
    
    % Cost function for velocity.
    costH(vStartIdx + i, vStartIdx + i) = velocityWeight + costH(vStartIdx + i, vStartIdx + i);
    costH(vStartIdx + i + 1, vStartIdx + i + 1) = velocityWeight + costH(vStartIdx + i + 1, vStartIdx + i + 1);
    costH(vHalfStartIdx + i, vHalfStartIdx + i) = 4 * velocityWeight;
    
    % Cost function for acceleration.
    costH(aStartIdx + i, aStartIdx + i) = (1 - velocityWeight) + costH(aStartIdx + i, aStartIdx + i);
    costH(aStartIdx + i + 1, aStartIdx + i + 1) = (1 - velocityWeight) + costH(aStartIdx + i + 1, aStartIdx + i + 1);
    costH(aHalfStartIdx + i, aHalfStartIdx + i) = 4 * (1 - velocityWeight);
end

% Add position constraints to some of collocation points.
posConstraintA = zeros(size(knots,1), numDecisionVars);
for i = 1:size(knots,1)
    posConstraintA(i,  xStartIdx + definedKnotIdx(i) - 1) = 1;
end

Aeq = [xCol; vCol; vHalfCol; posConstraintA];
Aboth = sparse(blkdiag(Aeq, Aeq));
bboth = sparse([zeros(3 * size(xCol,1),1); knots(:,1); zeros(3 * size(xCol,1),1); knots(:,2)]);
costHboth = sparse(blkdiag(costH, costH));
opt = optimoptions('quadprog');
opt.ConstraintTolerance = 1e-12;
X = quadprog(costHboth, [], [], [], Aboth, bboth, [], [], [], []);

x = X(1:numColPts);
vx = X(vStartIdx:vStartIdx + numColPts - 1);
vhx = X(vHalfStartIdx:vHalfStartIdx + numColPts - 2);

axCoeff = 1/3 * (2 * vx(1:end - 1) + 2 * vx(2:end) - 4 * vhx)./dt.^2;
bxCoeff = 1/2 * (-3 * vx(1:end - 1) + 4 * vhx - vx(2:end))./dt;
cxCoeff = vx(1:end - 1);
dxCoeff = x(1:end - 1);

ystart = length(X)/2;
y = X(1 + ystart:numColPts + ystart);
vy = X(vStartIdx + ystart:vStartIdx + numColPts - 1 + ystart);
vhy = X(vHalfStartIdx + ystart:vHalfStartIdx + numColPts - 2 + ystart);

ayCoeff = 1/3 * (2 * vy(1:end - 1, :) + 2 * vy(2:end, :) - 4 * vhy)./dt.^2;
byCoeff = 1/2 * (-3 * vy(1:end - 1, :) + 4 * vhy - vy(2:end, :))./dt;
cyCoeff = vy(1:end - 1, :);
dyCoeff = y(1:end - 1, :);

coefs = zeros(2 * length(axCoeff), 4);
coefs(1:2:end,:) = [axCoeff, bxCoeff, cxCoeff, dxCoeff];
coefs(2:2:end,:) = [ayCoeff, byCoeff, cyCoeff, dyCoeff];

splinepp = ppmak(allTimes, coefs, 2);

end

