SERVERS = {
    "basedpyright",
    "bashls",
    "clangd",
    "cssls",
    "jsonls",
    "jdtls",
    "lua_ls",
    "neocmake",
    "nil_ls",
    "rust_analyzer",
    "superhtml",
    "taplo",
    "vtsls",
    "yamlls",
}

vim.diagnostic.config({
    severity_sort = true,
    underline = { severity = { min = vim.diagnostic.severity.WARN } },
    virtual_text = false,
    virtual_lines = { current_line = true, },
    float = {
        border = "rounded",
        source = true,
        header = "",
        prefix = "",
    },
    signs = {
        text = {
            [vim.diagnostic.severity.ERROR] = "●",
            [vim.diagnostic.severity.WARN]  = "●",
            [vim.diagnostic.severity.INFO]  = "●",
            [vim.diagnostic.severity.HINT]  = "●",
        },
    },
})

local MiniCompletion = require("mini.completion")
local capabilities = vim.lsp.protocol.make_client_capabilities()
capabilities = vim.tbl_deep_extend("force", capabilities, MiniCompletion.get_lsp_capabilities())
vim.lsp.config("*", { capabilities = capabilities })

local grp = vim.api.nvim_create_augroup("LspKeymaps", { clear = true })
vim.api.nvim_create_autocmd("LspAttach", {
    group = grp,
    callback = function(args)
        require("lsp.keymaps").attach(args.buf)
    end,
})

vim.lsp.enable(SERVERS)
