vim.g.netrw_banner = 0
vim.g.netrw_bufsettings = "noma nomod nu nobl nowrap ro rnu"

vim.g.have_nerd_font = true

vim.o.number = true
vim.o.relativenumber = true
vim.o.signcolumn = "yes"

vim.o.shiftwidth = 4
vim.o.smarttab = true
vim.o.expandtab = true

vim.o.tabstop = 8
vim.o.softtabstop = 0

vim.o.list = true

vim.o.cmdheight = 2
vim.o.inccommand = "split"

vim.o.foldexpr = "v:lua.vim.treesitter.foldexpr()"
vim.o.foldlevel = 99;
vim.o.foldmethod = "expr"

vim.api.nvim_create_autocmd("TextYankPost", {
    callback = function()
        vim.hl.on_yank()
    end
})
