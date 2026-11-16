--- https://github.com/rebelot/kanagawa.nvim
require("kanagawa").setup({
    compile = false,
    undercurl = true,
    commentStyle = { italic = false },
    functionStyle = {},
    keywordStyle = { italic = false },
    statementStyle = { bold = false },
    typeStyle = {},
    transparent = false,
    dimInactive = false,
    terminalColors = true,
    theme = "wave",
    background = {
        dark = "wave",
        light = "lotus",
    },
})
vim.cmd("colorscheme kanagawa")
