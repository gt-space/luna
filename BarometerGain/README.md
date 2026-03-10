Code for selecting best fading memory filter gain.


# Installation Guide
I will assume you already have Julia installed (I am using 1.12.5), so you will just need to install the packages.

- `cd` into the folder you have this downloaded in (this folder MUST contain the Manifest.toml and Project.toml files)
- run `julia --project=.` in your terminal
- `]` to open Pkg
- `instantiate`

Currently the packages are:

- GLMakie: For data visualization and troubleshooting
- LoopVectorization: For multithreading and SIMD
- CSV: Read CSV data from sims team
- DataFrames: Used to convert CSV data into array form
- JLD2: Used to store data in the high-performance HDF5 file format without use of the HDF5 C library

# Performance Notes (IMPORTANT!!!!)
This shit is really fast on my desktop but that's because I have a CPU with AVX-512 instruction set (i7-11700k). You may not get as good performance as expected from CPUs without this instruction set.

Also make sure you set Julia to open with the correct # of cores for your system
