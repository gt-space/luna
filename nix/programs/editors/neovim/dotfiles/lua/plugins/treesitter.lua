vim.api.nvim_create_autocmd("FileType", {
    pattern = "*",
    callback = function(args)
        local buffer = args.buf
        local filetype = vim.bo[buffer].filetype

        local language = vim.treesitter.language.get_lang(filetype)
        if not language then
            return
        end

        local ok_add = vim.treesitter.language.add(language)
        if not ok_add then
            return
        end

        vim.treesitter.start(buffer, language)
    end,
})
