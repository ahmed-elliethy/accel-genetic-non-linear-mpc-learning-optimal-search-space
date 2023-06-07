# Accelerating genetic optimization of nonlinear model predictive control by learning optimal search space size
This repository contains programs to re-produce the results of the paper here https://arxiv.org/abs/2305.08094.
# License
Copyright (c) 2023 Ahmed Elliethy.

All rights reserved.

This software should be used, reproduced and modified only for informational and nonprofit purposes.

By downloading and/or using any of these files, you implicitly agree to all the terms of the license, as specified in the document LICENSE.txt (included in this package)

# Installation
The code requires matlab software installed and regression tool box. 

# Usage
This repository contains programs for both UAV and ground vehicle platforms. For each platform, the repository includes the following programs:
  - Motivation program.
  - Dataset creation program.
  - Experiments programs.
# Demo
To re-produce the results in the paper, run the following

1- For motivation results (in the motivation section), run
```
matlab Motivation_main.mlx
```

2- To re-generate the dataset, run
```
matlab UAV_DataSet_main.mlx
matlab VEC_dataset_main.mlx
```

1- To re-produce the results, run
```
matlab VEC_Main.m
matlab UAV_Main.m
```
The produced figures display the average computational time ,convergence percentage, and the performance metric for the fixed search space Genetic optimization (FG) technique and for our proposed adaptive search space Genetic optimization (AG) technique.
# Output
1- UAV :

  Fixed search space genetic optimization (FG):
   - Avg. comp. time (C) = 18.8 ms
   - Convergence percentage = 38.49 %
   - Performance metric (E) =  .77
   
  Adaptive search space genetic optimization (AG):
   - Avg. comp. time (C) = 8.987 ms
   - Convergence percentage = 76.54 %
   - Performance metric (E) =  .48

2- Vehicle :

  Fixed search space genetic optimization (FG):
   - Avg. comp. time (C) = 36.89 ms
   - Convergence percentage = 39.31 %
   - Performance metric (E) = 3.17
   
  Adaptive search space genetic optimization (AG):
   - Avg. comp. time (C) = 12.71 ms
   - Convergence percentage = 89.94 %
   - Performance metric (E) = 1.55
# Reference
```
@article{mostafa2023accelerating,
 title={Accelerating genetic optimization of nonlinear model predictive control by learning optimal search space size},
 author={Mostafa, Eslam and Aly, Hussein A and Elliethy, Ahmed},
 journal={arXiv preprint arXiv:2305.08094},
 year={2023}
}
```
