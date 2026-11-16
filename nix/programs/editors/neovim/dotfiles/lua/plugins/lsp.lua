vim.diagnostic.config({ virtual_text = true })

local MiniCompletion = require("mini.completion")
local capabilities = vim.lsp.protocol.make_client_capabilities()
capabilities = vim.tbl_deep_extend("force", capabilities, MiniCompletion.get_lsp_capabilities())
vim.lsp.config("*", { capabilities = capabilities })

vim.lsp.enable({
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
})
