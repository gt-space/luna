local M = {}

function M.attach(bufnr)
    local function map(mode, lhs, rhs, desc)
        vim.keymap.set(mode, lhs, rhs, {
            buffer = bufnr,
            silent = true,
            noremap = true,
            desc = desc,
        })
    end

    -- Core LSP
    map("n", "gD", vim.lsp.buf.declaration, "LSP: declaration")
    map("n", "gd", vim.lsp.buf.definition, "LSP: definition")
    map("n", "gI", vim.lsp.buf.implementation, "LSP: implementation")
    map("n", "gr", vim.lsp.buf.references, "LSP: references")
    map("n", "K", vim.lsp.buf.hover, "LSP: hover")
    map("n", "<leader>la", vim.lsp.buf.code_action, "LSP: code action")
    map("n", "<leader>lr", vim.lsp.buf.rename, "LSP: rename")
    map("n", "<leader>ls", vim.lsp.buf.signature_help, "LSP: signature help")

    -- Diagnostics
    map("n", "gl", vim.diagnostic.open_float, "Diagnostics: float")
    map("n", "<leader>lj", function()
        vim.diagnostic.jump { count = 1, float = true }
    end, "Diagnostics: next")
    map("n", "<leader>lk", function()
        vim.diagnostic.jump { count = -1, float = true }
    end, "Diagnostics: prev")
    map("n", "<leader>lq", vim.diagnostic.setloclist, "Diagnostics: loclist")

    -- Format
    map("n", "<leader>lf", function()
        vim.lsp.buf.format { bufnr = bufnr, async = true }
    end, "LSP: format")
end

return M
