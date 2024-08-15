# Servo (fs-server)

Servo is the main point of connection and communication in YJSP's software systems, a central server which handles procedure storage, flight computer communication, logging, data forwarding, GUI-to-FC interaction, and more.

## Getting Started

If not already installed on your system, install [Rust](https://www.rust-lang.org):

`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

Then, clone the repository. Remember, this requires authentication in the form of a [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) or [SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

```
# Using personal access token
git clone https://github-research.gatech.edu/YJSP/servo.git

# Using SSH key
git clone git@github-research.gatech.edu:YJSP/servo.git
```

Finally, install Servo by using the path to the project directory (by default, the path will be `./servo`). This will allow you to use the `servo` command globally.

`cargo install --path ./servo`

## Development

Welcome developers! For a quick rundown on developing for Servo, read below. For documentation of the API, check out [API.md](API.md). If you have any other questions, contact the RE for this project, [Jeff Shelton](https://github-research.gatech.edu/jshelton44). For documentation on Servo's internal library (mainly for Servo developers), clone this project and run `cargo doc --open`.

### Environment

Although you may use whatever code editor you prefer, [VSCode](https://code.visualstudio.com/) + the [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) plugin are excellent choices for Rust development. If you're new to Rust, the best reference to learn is the [Rust Book](https://doc.rust-lang.org/book), a free online book written by the Rust developers which is incredibly well-written and comprehensive, along with the typical [StackOverflow](https://stackoverflow.com).

### Guidelines

Servo needs to be 100% operational whenever it's in use, so here are a couple simple guidelines to help keep our code as stable as possible:

1. No panics! Avoid committing any code that uses `.unwrap()`, `.expect()`, `panic!()`, or any other function/method/macro that could cause the program to panic. There are a few rare exceptions to this rule, such as when obtaining a mutex lock or when you can **absolutely guarantee** that code will not panic under any circumstance.
2. No memory leaks! Objects that hold references to themselves or looping threads/async contexts which hold strong references to objects on the main thread should be avoided. (Thankfully, Rust makes it harder to make these kinds of mistakes, but async code has to be extra careful)
3. Recoverable errors! In general, if an error is recoverable, don't exit the program. Write functions that can handle all possible inputs, and if this is not possible for some function, make sure to document it.
4. Documentation! You may notice the `#![warn(missing_docs)]` flag in `src/lib.rs`. Since multiple people will be working on the same project, we need to thoroughly document everything so no one is confused.
5. Speed! Speed is very important to this project, especially if you're writing code that will be run thousands or millions of times in a loop. Use concurrency when possible and practical. Try to avoid mixing async concurrency (preferred) with multithreading.

Of course, everyone makes mistakes, which is why all pull requests must be reviewed by another developer before they are merged. We don't have a massive team and all of us are learning, so don't be afraid to submit and review pull requests, no matter your level of skill.

### Debugging

Effective debugging is essential on this project because this software is absolutely mission-critical. Using tools like **lldb** and **gdb** is recommended, especially for memory issues such as leaks. Segfaults should be less of an issue now since we are using Rust, but if you use _any_ unsafe code in a commit (such as interaction with hardware), please triple-check it for possible memory issues using one of these tools or another like them.

We also want to steadily introduce unit testing across YJSP software projects, including this one. If you are coding a non-trivial item of Servo, please write unit tests for that item. Debugging now means 10x less debugging later.
