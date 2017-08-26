const fp = {
  range(start, count) {
    return Array(...Array(count)).map((_, i) => start + i);
  },

  curry(uncurried, ...outerArgs) {
    if (outerArgs.length === 0) {
      return uncurried;
    }
    return function (...innerArgs) {
      uncurried(...outerArgs.concat(innerArgs));
    }
  },

  compose(...fxns) {
    if (fxs.length === 1) {
      return function (x) {
        return fxns[0](x);
      }
    }
    return function (x) {
      return fxns[0](compose(...fxns.slice(1))(0));
    }
  }

}