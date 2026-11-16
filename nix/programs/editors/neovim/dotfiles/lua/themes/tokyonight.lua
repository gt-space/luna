-- hhttps://github.com/folke/tokyonight.nvim 
require("tokyonight").setup({
    style = "night",
    light_style = "day",
    -- transparent = true,
    styles = {
        comments = { italic = false },
        keywords = { italic = false },
        functions = {},
        variables = {},

        sidebars = "dark",
        floats = "dark",
    },
})
vim.cmd("colorscheme tokyonight")
