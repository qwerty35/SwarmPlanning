\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj: - x0 - x1 - x2 + [ 2 x0 ^2 + 4 x0 * x1 + 4 x0 * x2 + 2 x1 ^2 + 4 x1 * x2
      + 2 x2 ^2 ] / 2
Subject To
 c1: - x0 + x1 + x2 <= 20
 c2: x0 - 3 x1 + x2 <= 30
Bounds
 0 <= x0 <= 40
End
