-- Load after nvim-lspconfig so its default command cannot replace these flags.
return {
    cmd = {
        "clangd",
        "--background-index",
        -- Nix compiler wrappers supply the standard library paths. Let clangd
        -- query the compiler from compile_commands.json (also for C and aliases).
        "--query-driver=" .. table.concat({
            "/nix/store/*/bin/clang",
            "/nix/store/*/bin/clang++",
            "/nix/store/*/bin/gcc",
            "/nix/store/*/bin/g++",
            "/nix/store/*/bin/cc",
            "/nix/store/*/bin/c++",
        }, ","),
    },
}
