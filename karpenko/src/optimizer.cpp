#include "optimizer.h"
#include <cmath>

namespace karpenko {

double coordinateDescent(std::vector<double>& x,
                         std::vector<double> step,
                         const std::vector<double>& tol,
                         const std::function<double(const std::vector<double>&)>& cost,
                         const CoordDescentOptions& opt) {
  double J = cost(x);
  const int D = (int)x.size();
  for (int iter = 0; iter < opt.max_iters; ++iter) {
    bool all_below = true;
    for (int d = 0; d < D; ++d) {
      if (std::fabs(step[d]) <= tol[d]) continue;
      all_below = false;
      // 试 +step
      std::vector<double> xp = x; xp[d] += step[d];
      double Jp = cost(xp);
      if (Jp < J) { x = xp; J = Jp; continue; }
      // 试 -step
      std::vector<double> xm = x; xm[d] -= step[d];
      double Jm = cost(xm);
      if (Jm < J) { x = xm; J = Jm; step[d] = -step[d]; continue; }  // 记住下降方向
      // 都不降 -> 收缩
      step[d] /= opt.shrink;
    }
    if (all_below) break;
  }
  return J;
}

}  // namespace karpenko
