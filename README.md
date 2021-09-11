# STRIDE: spectrahedral proximal gradient descent along vertices
A Solver for Large-Scale Rank-One Semidefinite Relaxations

## About
STRIDE is designed for solving high-order semidefinite programming (SDP) relaxations of nonconvex polynomial optimization problems (POPs) that admit rank-one optimal solutions. STRIDE is the first algorithmic framework that blends fast local search on the nonconvex POP with global descent on the convex SDP. Specifically, STRIDE follows a globally convergent trajectory driven by a proximal gradient method (PGM) for solving the SDP, while simultaneously probing long, but safeguarded, rank-one "strides", generated by fast nonlinear programming algorithms on the POP, to seek rapid descent. 

If you find STRIDE helpful or use it in your projects, please cite:

```bibtex
@article{Yang21arxiv-stride,
  title={STRIDE along Spectrahedral Vertices for Solving Large-Scale Rank-One Semidefinite Relaxations},
  author={Yang, Heng and Liang, Ling and Toh, Kim-Chuan and Carlone, Luca},
  journal={arXiv preprint arXiv:2105.14033},
  year={2021}
}
```

## Dependencies
In order to run the example code `example_quasar.m`, please download the following two packages and provide paths to them in `example_quasar.m`:
- [SDPNAL+](https://blog.nus.edu.sg/mattohkc/softwares/sdpnalplus/): STRIDE uses the ADMM+ subroutine in SDPNAL+ to warmstart.
- [Manopt](https://www.manopt.org/downloads.html): in `example_quasar.m`, STRIDE uses Manopt to perform local search to generate rank-one strides.


## Example
We provide a starting example about how to use STRIDE in the script `example_quasar.m`, you can simply run the script in Matlab.

For more examples of using STRIDE for machine perception applications, please navigate to the repo [CertifiablyRobustPerception](https://github.com/MIT-SPARK/CertifiablyRobustPerception).

## How to use STRIDE
The function signature for STRIDE is
```
[out,Xopt,yopt,Sopt] = PGDSDP(blk,At,b,C,X0,options)
```
where `PGDSDP` stands for _projected gradient descent_ in solving a generic SDP problem (which is the backbone of STRIDE). We now describe the detailed input and out of STRIDE.
### Input
- `blk,At,b,C`: standard SDP data in SDPT3 format. A standard SDP problem can be fully described by `blk,At,b,C`, where `blk` describes the sizes of the positive semidefinite constraints (i.e., blocks, we do not support other conic constraints such as second-order cone and nonnegative orthant), `At,b` describes the linear constraints, and `C` describes the linear cost function. `blk,At,C` should be Matlab cell arrays, while `b` should be a Matlab array. Please refer to the [SDPT3 user guide](https://blog.nus.edu.sg/mattohkc/softwares/sdpt3/) for details. We provide two example problem data for the QUASAR SDP in the subfolder `data`. If you are interested in how to generate standard SDP problem data from semidefinite relaxations of polynomial optimization problems, please navigate to the repo [CertifiablyRobustPerception](https://github.com/MIT-SPARK/CertifiablyRobustPerception).

- `X0`: a primal initial guess for the SDP problem. Set `X0 = []` if no initial guess is available. A good way of providing an initial primal guess is to use `fmincon` in Matlab to solve the original polynomial optimization problem (if the POP admits a manifold structure, Manopt should be preferred), obtain a local optimizer, and lift the local optimizer to a rank-one feasible point of the SDP. Please read our [paper](https://arxiv.org/abs/2105.14033) for more details. 

- `options`: a Matlab structure that provides more information. There are many available parameters in `options`, but there are two parameters that are required:
  - `options.rrFunName`: a string that provides the name of the Matlab function that implements a _local search_ scheme. For example, in the provided example `example_quasar.m`, we use `options.rrFunName = 'local_search_quasar'` to tell STRIDE that the function `local_search_quasar.m` implements the local search scheme.

  - `options.SDPNALpath`: a string that provides the path to the software package [SDPNAL+](https://blog.nus.edu.sg/mattohkc/softwares/sdpnalplus/). STRIDE uses the `admmplus` subroutine in SDPNAL+ to warmstart.
The other optional parameters are described in more details [below](https://github.com/MIT-SPARK/STRIDE#available-parameters).

### Output
- `Xopt,yopt,Sopt`: an (approximate) optimal solution to the SDP. In many cases, STRIDE can solve the SDP to very high accuracy (even better than MOSEK). The printout of STRIDE will show the KKT residuals at `Xopt,yopt,Sopt`.
- `out`: a Matlab structure that contains other information such as run history and runtime.

### Available parameters
We now list all the available but optional parameters in `options`:
- `options.S0`: a dual initial guess. Typically it is difficult to have a good guess on the dual variables. If not provided, STRIDE uses ADMM+ to generate dual initial guess. However, in some cases, one can exploit problem structure to provide clever dual initializations, please checkout our [paper](https://arxiv.org/abs/2109.03349) for details.

- `options.tolADMM`: accuracy tolerance for using ADMM+. We note that this is perhaps the most important parameter to tune for a fast performance. Setting `options.tolADMM` very low (e.g., `1e-12`) will ask ADMM+ to provide a very accurate warmstart (in the price of more ADMM+ iterations and runtime) so that the main STRIDE algorithm will converge very fast. Setting `options.tolADMM` very high (e.g., `1e-4`) will not require an accurate warmstart from ADMM+ (so very few ADMM+ iterations and less runtime), but it may take many STRIDE main PGD iterations. We recommend tuning this parameter for each specific problem. For the QUASAR examples in this repo, `options.tolADMM = 1e-4` works very well. 

- `options.maxiterADMM`: maximum ADMM+ iterations, default `1e4`.

- `options.tolPGD`: accuracy tolerance for STRIDE, in terms of maximum relative KKT residual, default `1e-6`.

- `options.pgdStepSize`: step size for projected gradient descent. We recommend setting `options.pgdStepSize = 10`.

- `options.maxiterPGD`: maximum outer iterations of STRIDE (in performing projected gradient descent), default 10.

- `options.lbfgsmemory`: memory of L-BFGS, default 10.

- `options.maxiterLBFGS`: maximum iterations of L-BFGS, default 1000.

- `options.lbfgseps`: boolean value to decide if using inexactness in L-BFGS (what we call modified L-BFGS), default `options.lbfgseps = true`. In practice we found this does not have significant effect on the convergence speed.

- `options.rrOpt`: a array that contains the indices of the eigenvectors to be rounded in local search, default `options.rrOpt = 1:3` and STRIDE generates rounded hypotheses from the leading 3 eigenvectors.

- `options.rrPar`: a Matlab structure that contains all user-defined information needed to perform local search. For a template about how to implement a local search scheme, please see [below](https://github.com/MIT-SPARK/STRIDE#implement-your-local-search-scheme).


## Implement your local search scheme
Coming soon...

## Acknowledgements
STRIDE is implemented by [Heng Yang](https://hankyang.mit.edu/) (MIT) and [Ling Liang](https://blog.nus.edu.sg/liangling/) (NUS). We would like to thank the feedback and resources from [Prof. Kim-Chuan Toh](https://blog.nus.edu.sg/mattohkc/) (NUS), and [Prof. Luca Carlone](https://lucacarlone.mit.edu/) (MIT).







