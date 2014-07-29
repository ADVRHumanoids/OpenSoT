function [stat, feas, cmpl] = qpresidual(B, b, C, cl, cu, x, y)

stat = norm(B * x + C' * y + b, Inf);
Cx = C * x;
resl = cl - Cx;
resu = Cx - cu;
feas = max([0; resl; resu]);
cmpl = norm(min([-min(y,0); max(y,0)], abs([resl; resu])), Inf);

