function d = pointToLine(pt, v1, v2)
% function to calculate distance between point 'pt' and line defined by two vertices
% 'v1' and 'v2'

      a = v1 - v2;
      b = pt - v2;
      d = norm(cross(a,b)) / norm(a);
end