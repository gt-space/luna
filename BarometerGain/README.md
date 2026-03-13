Code for selecting best fading memory filter gain.

# Installation Guide
I will assume you already have Julia installed, so you will just need to install the packages. I suggest pairing this with juliaup as well as VSCode's Julia extension. I can only guarantee the code works in 1.21.5 (the version I am using), but other versions should be fine.

- `alt + J` `alt + O` to open Julia REPL
- in REPL, `cd` into the folder you have this downloaded in (this folder MUST contain the Manifest.toml and Project.toml files)
- `]` to open Pkg mode (this is Julia's file manager)
- run `activate .` in your REPL
- `instantiate`

Currently the packages are:

- GLMakie: For data visualization and troubleshooting
- LoopVectorization: For multithreading and SIMD
- CSV: Read CSV data from sims team
- DataFrames: Used to convert CSV data into array form
- JLD2: Used to store data in the high-performance HDF5 file format without use of the HDF5 C library

# Usage
Take the CSV data given by sims team and convert into altitude values, before performing computations on those.

# Performance Notes (IMPORTANT!!!!)
This shit is really fast on my desktop but that's because I have a CPU with AVX-512 instruction set (i7-11700k). You may not get as good performance as expected from CPUs without this instruction set.

Also make sure you set Julia to open with the correct # of cores for your system
