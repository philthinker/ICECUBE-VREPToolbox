
===========================================================================
BP-AR-HMM (beta process autoregressive hidden Markov model) Matlab Software
===========================================================================

Copyright (C) 2009, Emily B. Fox and Erik B. Sudderth.
(ebfox[at]alum[dot]mit[dot]edu and sudderth[at]cs[dot]brown[dot]edu)

This software package includes several Matlab scripts and
auxiliary functions, which implement MCMC sampling algorithms 
for the model described in the following publication:
  Sharing Features among Dynamical Systems with Beta Processes
  E. B. Fox, E. B. Sudderth, M. I. Jordan, and A. S. Willsky
  Advances in Neural Information Processing Systems, vol. 22, 2010.
Please cite this paper in any publications using the BP-AR-HMM package.

See also:
  Bayesian Nonparametric Learning of Complex Dynamical Phenomena
  E. B. Fox
  Ph.D. Thesis, July, 2009.

========================================================================
Package Organization and Documentation
========================================================================

Summary of BP-AR-HMM package contents:

IBPHMMinference.m:
  Main inference script using the birth-death RJMCMC sampler for unique features.
IBPHMMinference_PoissonProp.m:
  Main inference script using Poisson proposal for unique features.
/utilities:  
  Script runstuff.m with example inputs to main inference script, along
  with various other functions used to create necessary structures, etc.
/relabeler:  
  Code to perform optimal mapping between true and estimated mode sequences.

========================================================================
Setup and Usage Examples
========================================================================

For an example of sparse feature extraction, see runstuff.m.
To use the BP-AR-HMM code, you must first take two steps:
1) Install Minka's lightspeed toolbox and add directory to path:
	http://research.microsoft.com/~minka/software/lightspeed/
2) Add /relabeler and /utilities directory to path

========================================================================
Acknowledgments
========================================================================

Portions of the package were adapted from Yee Whye Teh's
"Nonparametric Bayesian Mixture Models" package, release 1.
Available from:  http://www.gatsby.ucl.ac.uk/~ywteh

========================================================================
Copyright & License
========================================================================

Copyright (C) 2009, Emily B. Fox and Erik B. Sudderth.

http://web.mit.edu/ebfox/www/

Permission is granted for anyone to copy, use, or modify these
programs and accompanying documents for purposes of research or
education, provided this copyright notice is retained, and note is
made of any changes that have been made.

These programs and documents are distributed without any warranty,
express or implied.  As the programs were written for research
purposes only, they have not been tested to the degree that would be
advisable in any important application.  All use of these programs is
entirely at the user's own risk.