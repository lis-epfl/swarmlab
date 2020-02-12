function pts = translate(pts, pn, pe, pd)
% TRANSLATE - Apply translation to a vector of points

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end