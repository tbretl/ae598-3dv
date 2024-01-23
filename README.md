# AE 598 3DV (An Invitation to 3-D Vision)

## Getting started

Create a directory in which to put all of your work this semester. Let's suppose you call this directory `AE598-3DV-Dev`. Clone the repo you're looking at into this directory, as usual:
```zsh
git clone https://github.com/tbretl/ae598-3dv.git
```


### Install conda

I use [miniforge](https://github.com/conda-forge/miniforge) to install `conda`.

I do not suggest you use [anaconda](https://anaconda.org) for this purpose - it causes trouble. If you already have anaconda installed, I suggest you remove it.

It is ok to use [miniconda](https://docs.conda.io/projects/miniconda/) - I used to do this - but if you do, I suggest that you configure your conda environment to use `conda-forge` as the only channel. To do this, you'd run the following commands just after creating and activating an empty envirionment:
```zsh
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
```
This configuration is done by default in every environment if you use `miniforge`.

I use `mamba` as a drop-in replacement for `conda` (all the cool kids do this). Both come with `miniforge`. So, the instructions that follow will all use `mamba`. You can use `conda` instead if you like.

### Create a conda environment

This will get us started:
```zsh
mamba create -n ae598-3dv
mamba activate ae598-3dv
mamba install python=3 numpy scipy sympy matplotlib notebook ipywidgets ipympl opencv cmake eigen
```
We will add more packages as we need them.

### Install symforce

[SymForce](https://github.com/symforce-org/symforce) is a library for nonlinear least-squares minimization. I like it because, unlike other libraries of this sort, it has a Python interface that is reasonably easy to use and is reasonably well documented. It also allows you, if you like, to autogenerate C++ code that is competitive with other state-of-the-art solvers (e.g., [ceres](http://ceres-solver.org), [g2o](https://github.com/RainerKuemmerle/g2o), and [GTSAM](https://gtsam.org)).

Although SymForce can be installed using `pip`, I suggest you [build it from source](https://github.com/symforce-org/symforce#build-from-source). There are two reasons for this. First, some recent bug fixes haven't made it to the [version that's available on PyPi](https://pypi.org/project/symforce/) yet. Second, the C++ headers are only available if you build and install from source.

It takes just a couple minutes to follow [the instructions to build from source](https://github.com/symforce-org/symforce#build-from-source) on my mac. First, I installed [brew](https://brew.sh) and used brew to install `gmp` (`brew install gmp`). Then, I cloned the SymForce repository (putting it in `AE598-3DV-Dev`) and used `pip` to compile and install (don't forget to activate your `ae598-3dv` environment first):
```zsh
git clone https://github.com/symforce-org/symforce.git
cd symforce
pip install .
cd ..
```
The details will be a little different if you use Linux or Windows.

Don't forget to [verify your installation](https://github.com/symforce-org/symforce#verify-your-installation) after you install SymForce.
