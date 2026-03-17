Code for selecting best fading memory filter gain.

# Installation Guide
I will assume you already have Julia installed, so you will just need to install the packages. I suggest pairing this with juliaup as well as VSCode's Julia extension. I can only guarantee the code works in 1.21.5 (the version I am using), but other versions should be fine.

- `alt + J` `alt + O` to open Julia REPL
- In REPL, `cd("[insert_some_filepath]")` into the path of the folder you have this downloaded in (this folder MUST contain the Manifest.toml and Project.toml files).
- You can use `pwd()` to check that your path is correct.
- If you hit `]` with the REPL open, you enter Pkg mode (which is Julia's file manager). Use backspace to exit Pkg mode at any time.
- While in Pkg, run `activate .` to open a new project environment.
- Also while in Pkg, `instantiate` to build the required packages in your local project environment.

Currently the packages are:

- LoopVectorization: For multithreading and SIMD
- GLMakie: For data visualization and troubleshooting
- JLD2: Used to store data in the high-performance HDF5 file format without use of the HDF5 C library
- CSV: Read CSV data from sims team
- DataFrames: Used to convert CSV data into array form

# Usage
Take the CSV data given by sims team and convert into altitude values, before performing computations on those.

# Performance Notes (IMPORTANT!!!!)
This shit is really fast on my desktop but that's because I have a CPU with AVX-512 instruction set (i7-11700k). You may not get as good performance as expected from CPUs without this instruction set.

Also make sure you set Julia to open with the correct # of cores for your system
