// optimizer.h —— Karpenko 的坐标下降（直接目标函数求值）。
// 逐参数试探：+step 若下降则接受；否则 -step；再否则该参数 step/=3。
// 直到所有参数步长低于阈值或达到最大迭代。返回最终代价。
#pragma once
#include <functional>
#include <vector>

namespace karpenko {

struct CoordDescentOptions {
  int max_iters = 200;
  double shrink = 3.0;
};

// x: 初值/输出；step: 各维初始步长；tol: 各维步长收敛阈值；cost(x)->标量。
double coordinateDescent(std::vector<double>& x,
                         std::vector<double> step,
                         const std::vector<double>& tol,
                         const std::function<double(const std::vector<double>&)>& cost,
                         const CoordDescentOptions& opt = {});

}  // namespace karpenko
